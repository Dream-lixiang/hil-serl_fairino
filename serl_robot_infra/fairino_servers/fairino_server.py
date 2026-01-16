"""
Fairino (法奥) robot control server for HIL-SERL.
Strictly mirrors the standard server API for compatibility.

Endpoints:
- /pose: POST {"arr": [x, y, z, qx, qy, qz, qw]}
- /getstate: POST -> {"pose": [7], "q": [6], "gripper_pos": float, "gripper_state": {}}
- /jointreset: POST
- /close_gripper: POST
- /open_gripper: POST
- /clearerr: POST
"""

from __future__ import annotations

import os
import sys
import threading
import time
import copy
from dataclasses import dataclass
from typing import Any, Optional, Tuple

import numpy as np
from flask import Flask, jsonify, request
from scipy.spatial.transform import Rotation as R
from absl import app, flags


FLAGS = flags.FLAGS

flags.DEFINE_string("robot_ip", "192.168.58.6", "Fairino controller IP")
flags.DEFINE_string("flask_url", "127.0.0.1", "Host IP for the Flask server")
flags.DEFINE_integer("port", 5000, "Port for the Flask server")

flags.DEFINE_float("cmdT", 0.008, "Servo command period in seconds.")
flags.DEFINE_enum("servo_mode", "cart", ["cart", "joint"], "Streaming mode.")

flags.DEFINE_integer("tool", 0, "Tool index")
flags.DEFINE_integer("user", 0, "User frame index")
flags.DEFINE_list("reset_joint_target", [-90.0, -90.0, -90.0, -90.0, 90.0, 0.0], "Target joint angles [deg] for /jointreset.")
flags.DEFINE_integer("dof", 6, "Robot DoF.")


def _maybe_call(robot: Any, name: str, *args, **kwargs):
    fn = getattr(robot, name, None)
    if fn is None:
        return None
    return fn(*args, **kwargs)


def _first_available_call(robot: Any, candidates: list[Tuple[str, tuple, dict]]) -> Optional[Any]:
    for name, args, kwargs in candidates:
        if hasattr(robot, name):
            return _maybe_call(robot, name, *args, **kwargs)
    return None


@dataclass
class FairinoState:
    tcp_pose_m_quat: np.ndarray  # (7,) meters + quat xyzw
    q_deg: np.ndarray  # (dof,) degrees
    gripper_pos: float  # scalar in [0,1]
    raw_gripper_state: dict


class FairinoServoController:
    """
    Background thread that keeps sending ServoCart/ServoJ commands 
    to maintain the robot's position or track a target.
    """

    def __init__(
        self, robot_ip: str, cmdT: float, servo_mode: str, tool: int, user: int, dof: int = 6
    ):
        # Vendored SDK import
        vendored_root = os.path.join(os.path.dirname(__file__), "Fairino_Arm")
        if os.path.isdir(vendored_root) and vendored_root not in sys.path:
            sys.path.insert(0, vendored_root)

        from fairino import Robot  # type: ignore

        self.robot = Robot.RPC(robot_ip)
        self.cmdT = float(cmdT)
        self.servo_mode = servo_mode
        self.tool = int(tool)
        self.user = int(user)
        self.dof = int(dof)

        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Targets
        self._target_pose_m_quat: Optional[np.ndarray] = None
        self._target_desc_pos_mm_deg: Optional[np.ndarray] = None
        self._target_joint_deg: Optional[np.ndarray] = None
        self._servoj_id = 0

        # Cached state
        self._last_state = FairinoState(
            tcp_pose_m_quat=np.zeros((7,), dtype=np.float64),
            q_deg=np.zeros((self.dof,), dtype=np.float64),
            gripper_pos=0.0,
            raw_gripper_state={},
        )
        self.get_state()

    def start(self):
        if self._thread is not None:
            return
        print("Starting Servo Loop Thread...")
        self._running = True
        self._thread = threading.Thread(target=self._servo_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None
        _maybe_call(self.robot, "ServoMoveEnd")

    def close(self):
        self.stop()
        _maybe_call(self.robot, "CloseRPC")

    def set_target_pose_m_quat(self, pose7: np.ndarray):
        """Update the target for the servo loop."""
        pose7 = np.asarray(pose7, dtype=np.float64).reshape(7)
        xyz_mm = pose7[:3] * 1000.0
        euler_deg = R.from_quat(pose7[3:]).as_euler("xyz", degrees=True)
        desc_pos_mm_deg = np.concatenate([xyz_mm, euler_deg]).astype(np.float64)
        with self._lock:
            self._target_pose_m_quat = pose7
            self._target_desc_pos_mm_deg = desc_pos_mm_deg

    def joint_reset(self, reset_q_deg: list[float]) -> int:
        """Interrupt servo loop, move to joint target, resume servo."""
        # 1. Pause servo
        was_running = self._running
        if was_running:
            # We don't stop the thread, just let it idle or we explicitly end servo mode
            pass 

        # Ideally: stop servo move, do MoveJ, then restart
        # But `ServoMoveEnd` might be needed.
        _maybe_call(self.robot, "ServoMoveEnd")
        time.sleep(0.05)
        
        _maybe_call(self.robot, "ResetAllError")

        # 2. MoveJ
        # input("Ready to move joint reset?")
        print(f"Executing MoveJ to: {reset_q_deg}")
        ret = _maybe_call(
            self.robot,
            "MoveJ",
            joint_pos=list(map(float, reset_q_deg)),
            tool=self.tool,
            user=self.user,
            vel=20.0,
            blendT=-1.0,
        )
        print(f"MoveJ returned: {ret}")

        # Wait for motion done
        while True:
            res = _maybe_call(self.robot, "GetRobotMotionDone")
            if res is not None and res[1] == 1:
                break
            time.sleep(0.1)
        time.sleep(1)
        
        # 3. Clear targets so we don't snap back
        with self._lock:
            self._target_pose_m_quat = None
            self._target_desc_pos_mm_deg = None
            self._target_joint_deg = None
            
        # 4. Resume
        _maybe_call(self.robot, "ServoMoveStart")
        return int(ret) if ret is not None else -1

    def get_state(self) -> FairinoState:
        # 1. Joint
        q = None
        res_q = _first_available_call(
            self.robot,
            [
                ("GetActualJointPosDegree", (0,), {}),
                ("GetActualJointPosDegree", (1,), {}),
                ("GetActualJointPos", (0,), {}),
            ],
        )
        if res_q is not None:
            err, data = res_q
            if int(err) == 0:
                q = np.asarray(data, dtype=np.float64)

        # 2. Pose
        tcp_pose_m_quat = None
        res_p = _first_available_call(
            self.robot,
            [
                ("GetActualTCPPose", (0,), {}),
                ("GetActualTCPPose", (1,), {}),
                ("GetActualToolPose", (0,), {}),
            ],
        )
        if res_p is not None:
            err, pose6 = res_p
            if int(err) == 0:
                pose6 = np.asarray(pose6, dtype=np.float64).reshape(6)
                xyz_m = pose6[:3] / 1000.0
                quat = R.from_euler("xyz", pose6[3:], degrees=True).as_quat()
                tcp_pose_m_quat = np.concatenate([xyz_m, quat]).astype(np.float64)

        # If we failed to read pose, use last known or target
        if tcp_pose_m_quat is None:
            with self._lock:
                if self._target_pose_m_quat is not None:
                    tcp_pose_m_quat = self._target_pose_m_quat.copy()
        if tcp_pose_m_quat is None:
            tcp_pose_m_quat = self._last_state.tcp_pose_m_quat.copy()

        q_deg = (
            np.asarray(q, dtype=np.float64).reshape(-1)[: self.dof].copy()
            if q is not None
            else self._last_state.q_deg.copy()
        )

        st = FairinoState(
            tcp_pose_m_quat=tcp_pose_m_quat,
            q_deg=q_deg,
            gripper_pos=0.0, # Placeholder, updated if needed
            raw_gripper_state={},
        )
        self._last_state = st
        return st

    def _servo_loop(self):
        _maybe_call(self.robot, "ServoMoveStart")
        while self._running:
            t0 = time.time()
            if self.servo_mode == "cart":
                with self._lock:
                    target = (
                        None
                        if self._target_desc_pos_mm_deg is None
                        else self._target_desc_pos_mm_deg.copy()
                    )
                if target is not None:
                    # Debug: print target every 100 iterations (approx 1s) to avoid spam
                    # if int(time.time() * 100) % 100 == 0:
                    #    print(f"ServoCart target: {target}")

                    ret = _maybe_call(
                        self.robot,
                        "ServoCart",
                        mode=0,
                        desc_pos=target.tolist(),
                        pos_gain=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
                        acc=0.0,
                        vel=3.0,
                        cmdT=self.cmdT,
                        filterT=0.04,
                        gain=0.0,
                    )
                    if ret != 0:
                        print(f"ServoCart error: {ret}")
            # ... joint mode omitted for brevity unless needed ...

            dt = time.time() - t0
            time.sleep(max(0.0, self.cmdT - dt))


class RMServerCompat:
    """
    Wrapper that exposes the exact methods expected by the Flask routes 
    in standard server style.
    """
    def __init__(self, controller: FairinoServoController):
        self.ctrl = controller
        # Initialize internal state fields to match RMServer
        self.pos = [0.0]*7
        self.q = [0.0]*6
        self.gripper_pos = 0.0
        self.gripper_state = {}

    def move(self, pose_arr: list):
        """Matches RMServer.move([x,y,z,qx,qy,qz,qw])"""
        self.ctrl.set_target_pose_m_quat(np.array(pose_arr))

    def get_gripper_state(self):
        # Fairino might not have a gripper, return success dummy
        return True

    def reset_joint(self):
        # Use FLAGS for target
        reset_q = [float(x) for x in FLAGS.reset_joint_target]
        self.ctrl.joint_reset(reset_q[: FLAGS.dof])

    def clear(self):
        _maybe_call(self.ctrl.robot, "ClearError")

    def open_gripper(self):
        # NO-OP for now or implementing if hardware exists
        pass

    def close_gripper(self):
        pass

    def update_internal_state(self):
        """Syncs controller state to local fields."""
        st = self.ctrl.get_state()
        self.pos = st.tcp_pose_m_quat.tolist()
        self.q = st.q_deg.tolist()
        self.gripper_pos = st.gripper_pos
        self.gripper_state = st.raw_gripper_state


def main(_):
    webapp = Flask(__name__)

    ctrl = FairinoServoController(
        robot_ip=FLAGS.robot_ip,
        cmdT=FLAGS.cmdT,
        servo_mode=FLAGS.servo_mode,
        tool=FLAGS.tool,
        user=FLAGS.user,
        dof=FLAGS.dof,
    )
    ctrl.start()
    
    # Compat wrapper
    robot_server = RMServerCompat(ctrl)

    @webapp.route("/pose", methods=["POST"])
    def pose():
        # Expected: {"arr": [7 floats]}
        arr = request.json.get("arr", [])
        if len(arr) == 7:
            robot_server.move(arr)
            return "Moved"
        return "Invalid pose", 400

    @webapp.route("/getstate", methods=["POST"])
    def getstate():
        robot_server.update_internal_state()
        return jsonify({
            "pose": robot_server.pos,
            "q": robot_server.q,
            "gripper_pos": robot_server.gripper_pos,
            "gripper_state": robot_server.gripper_state,
        })

    @webapp.route("/jointreset", methods=["POST"])
    def jointreset():
        robot_server.reset_joint()
        return "Reset Joint"

    @webapp.route("/clearerr", methods=["POST"])
    def clearerr():
        robot_server.clear()
        return "Clear"

    @webapp.route("/open_gripper", methods=["POST"])
    def open_gripper():
        robot_server.open_gripper()
        return "Opened"

    @webapp.route("/close_gripper", methods=["POST"])
    def close_gripper():
        robot_server.close_gripper()
        return "Closed"
    
    # Extra endpoints for compatibility
    @webapp.route("/set_gripper_position", methods=["POST"])
    def set_gripper_position():
        return jsonify({"success": True})

    try:
        webapp.run(host=FLAGS.flask_url, port=FLAGS.port)
    finally:
        ctrl.close()


if __name__ == "__main__":
    app.run(main)
