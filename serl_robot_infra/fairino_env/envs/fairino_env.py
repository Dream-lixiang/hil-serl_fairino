"""
Gym Interface for Fairino arms (法奥).

This env controls the robot via the local Flask server in
`serl_robot_infra/fairino_servers/fairino_server.py`.

Notes (per your request):
- If gripper / cameras are not configured, keep them as NO-OP (do not block training).
- Only the arm motion endpoints are assumed to exist.
"""

from __future__ import annotations

import copy
import os
from collections import OrderedDict
from datetime import datetime
from typing import Dict

import gymnasium as gym
import numpy as np
import requests
from scipy.spatial.transform import Rotation, Slerp
import time



def _euler_2_quat(euler_xyz: np.ndarray) -> np.ndarray:
    """Euler xyz (rad) -> quat xyzw."""
    return Rotation.from_euler("xyz", np.asarray(euler_xyz, dtype=np.float64)).as_quat()


class DefaultEnvConfig:
    """Default configuration FairinoEnv."""

    SERVER_URL: str = "http://127.0.0.1:5000/"

    # No cameras by default (your setup currently only has the arm).
    REALSENSE_CAMERAS: Dict = {}
    IMAGE_CROP: dict[str, callable] = {}

    TARGET_POSE: np.ndarray = np.zeros((6,))
    GRASP_POSE: np.ndarray = np.zeros((6,))
    REWARD_THRESHOLD: np.ndarray = np.zeros((6,))
    ACTION_SCALE = np.ones((3,))
    RESET_POSE = np.zeros((6,))

    RANDOM_RESET = False
    RANDOM_XY_RANGE = (0.0,)
    RANDOM_RZ_RANGE = (0.0,)

    # Safety bounds (kept for parity; clip is a no-op unless you customize it)
    ABS_POSE_LIMIT_HIGH = np.zeros((6,))
    ABS_POSE_LIMIT_LOW = np.zeros((6,))

    COMPLIANCE_PARAM: Dict[str, float] = {}
    RESET_PARAM: Dict[str, float] = {}
    PRECISION_PARAM: Dict[str, float] = {}

    DISPLAY_IMAGE: bool = False
    GRIPPER_SLEEP: float = 1.0
    MAX_EPISODE_LENGTH: int = 100
    JOINT_RESET_PERIOD: int = 0


class FairinoEnv(gym.Env):
    """Gym env backed by Fairino server."""

    def __init__(
        self,
        hz=10,
        fake_env=False,
        save_video=False,
        config: DefaultEnvConfig = None,
        set_load=False,
    ):
        # Ensure fake_env is set early
        self.fake_env = fake_env

        if config is None:
            config = DefaultEnvConfig()

        self.action_scale = np.asarray(config.ACTION_SCALE, dtype=np.float64)
        self._TARGET_POSE = np.asarray(config.TARGET_POSE, dtype=np.float64)
        self._RESET_POSE = np.asarray(config.RESET_POSE, dtype=np.float64)
        self._REWARD_THRESHOLD = np.asarray(config.REWARD_THRESHOLD, dtype=np.float64)
        self.url = config.SERVER_URL if config.SERVER_URL.endswith("/") else (config.SERVER_URL + "/")
        self.config = config
        self.max_episode_length = int(config.MAX_EPISODE_LENGTH)
        self.display_image = bool(config.DISPLAY_IMAGE)
        self.gripper_sleep = float(config.GRIPPER_SLEEP)

        # Convert last 3 elements from euler to quat, from size (6,) to (7,)
        self.resetpos = np.concatenate([config.RESET_POSE[:3], _euler_2_quat(config.RESET_POSE[3:])])

        self._update_currpos()
        self.last_gripper_act = time.time()
        self.lastsent = time.time()
        self.randomreset = bool(config.RANDOM_RESET)
        self.random_xy_range = config.RANDOM_XY_RANGE
        self.random_rz_range = config.RANDOM_RZ_RANGE
        self.hz = int(hz)
        self.joint_reset_cycle = int(config.JOINT_RESET_PERIOD)

        self.save_video = bool(save_video)
        if self.save_video:
            self.recording_frames = []

        # Boundary box (kept for parity)
        self.xyz_bounding_box = gym.spaces.Box(
            np.asarray(config.ABS_POSE_LIMIT_LOW[:3], dtype=np.float64),
            np.asarray(config.ABS_POSE_LIMIT_HIGH[:3], dtype=np.float64),
            dtype=np.float64,
        )
        self.rpy_bounding_box = gym.spaces.Box(
            np.asarray(config.ABS_POSE_LIMIT_LOW[3:], dtype=np.float64),
            np.asarray(config.ABS_POSE_LIMIT_HIGH[3:], dtype=np.float64),
            dtype=np.float64,
        )

        # Action space (SERL expects 7-dim: xyz + rot + gripper)
        self.action_space = gym.spaces.Box(np.ones((7,), dtype=np.float32) * -1, np.ones((7,), dtype=np.float32))

        # Observation space (minimal)
        self.observation_space = gym.spaces.Dict(
            {
                "state": gym.spaces.Dict(
                    {
                        "tcp_pose": gym.spaces.Box(-np.inf, np.inf, shape=(7,)),
                        "gripper_pose": gym.spaces.Box(-1, 1, shape=(1,)),
                    }
                ),
                "images": gym.spaces.Dict(
                    {key: gym.spaces.Box(0, 255, shape=(128, 128, 3), dtype=np.uint8) for key in config.REALSENSE_CAMERAS}
                ),
            }
        )

        self.cycle_count = 0
        self.curr_path_length = 0

        if fake_env:
            return

        self.cap = None
        if len(config.REALSENSE_CAMERAS):
            self.init_cameras(config.REALSENSE_CAMERAS)

        # Optional GUI / keyboard terminate logic skipped here to keep deps light.

    def clip_safety_box(self, pose: np.ndarray) -> np.ndarray:
        """Clip the pose to be within the safety box (NO-OP by default)."""
        return pose

    def step(self, action: np.ndarray) -> tuple:
        """Standard gym step function."""
        start_time = time.time()
        action = np.asarray(action, dtype=np.float64).reshape(-1)

        xyz_delta = action[:3]
        self.nextpos = self.currpos.copy()
        self.nextpos[:3] = self.nextpos[:3] + xyz_delta * self.action_scale[0]

        # Orientation delta (uses delta-euler)
        self.nextpos[3:] = (
            Rotation.from_euler("xyz", action[3:6] * self.action_scale[1]) * Rotation.from_quat(self.currpos[3:])
        ).as_quat()

        # Gripper is currently not configured; keep endpoint calls best-effort/no-op.
        gripper_action = float(action[6] * self.action_scale[2])
        self._send_gripper_command(gripper_action)
        self._send_pos_command(self.clip_safety_box(self.nextpos))

        self.curr_path_length += 1
        dt = time.time() - start_time
        time.sleep(max(0, (1.0 / self.hz) - dt))

        self._update_currpos()
        obs = self._get_obs()
        reward = self.compute_reward(obs)
        done = self.curr_path_length >= self.max_episode_length or reward
        return obs, int(reward), done, False, {"succeed": reward}

    def reset(self, joint_reset=False, **kwargs):
        self.last_gripper_act = time.time()

        if self.save_video:
            self.save_video_recording()

        self.cycle_count += 1
        if self.joint_reset_cycle != 0 and self.cycle_count % self.joint_reset_cycle == 0:
            self.cycle_count = 0
            joint_reset = True

        self._recover()
        self.go_to_reset(joint_reset=joint_reset)
        self._recover()
        self.curr_path_length = 0

        self._update_currpos()
        obs = self._get_obs()
        return obs, {"succeed": False}

    def save_video_recording(self):
        try:
            if not self.save_video or not len(self.recording_frames):
                return

            import cv2

            if not os.path.exists("./videos"):
                os.makedirs("./videos")
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

            for camera_key in self.recording_frames[0].keys():
                video_path = f"./videos/fairino_{camera_key}_{timestamp}.mp4"
                first_frame = self.recording_frames[0][camera_key]
                height, width = first_frame.shape[:2]

                video_writer = cv2.VideoWriter(
                    video_path,
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    10,
                    (width, height),
                )

                for frame_dict in self.recording_frames:
                    video_writer.write(frame_dict[camera_key])
                video_writer.release()

            self.recording_frames.clear()
        except Exception:
            pass

    def _recover(self):
        """Internal function to recover the robot from error state."""
        requests.post(self.url + "clearerr", timeout=1.0)

    def go_to_reset(self, joint_reset=False):
        """Reset routine: joint reset or Cartesian interpolate."""
        self._update_currpos()

        if joint_reset:
            requests.post(self.url + "jointreset", timeout=30.0)
            time.sleep(1.0)
            return

        if self.randomreset:
            reset_pose = self.resetpos.copy()
            reset_pose[:2] += np.random.uniform(-self.random_xy_range, self.random_xy_range, (2,))
            euler_random = self._RESET_POSE[3:].copy()
            euler_random[-1] += np.random.uniform(-self.random_rz_range, self.random_rz_range)
            reset_pose[3:] = _euler_2_quat(euler_random)
            self.interpolate_move(reset_pose, timeout=3)
        else:
            self.interpolate_move(self.resetpos.copy(), timeout=3)

    def interpolate_move(self, goal: np.ndarray, timeout: float):
        """Move the robot to the goal position with linear interpolation + quaternion SLERP."""
        goal = np.asarray(goal, dtype=np.float64)
        if goal.shape == (6,):
            goal = np.concatenate([goal[:3], _euler_2_quat(goal[3:])])

        steps = max(1, int(timeout * self.hz))
        self._update_currpos()

        position_path = np.linspace(self.currpos[:3], goal[:3], steps)
        key_rots = Rotation.from_quat([self.currpos[3:], goal[3:]])
        slerp = Slerp([0, 1], key_rots)
        rot_path = slerp(np.linspace(0, 1, steps)).as_quat()

        for i in range(steps):
            p = np.concatenate([position_path[i], rot_path[i]])
            self._send_pos_command(p)
            time.sleep(1 / self.hz)

        self.nextpos = np.concatenate([position_path[-1], rot_path[-1]])
        self._update_currpos()

    def compute_reward(self, obs) -> bool:
        current_pose = obs["state"]["tcp_pose"]
        current_rot = Rotation.from_quat(current_pose[3:]).as_matrix()
        target_rot = Rotation.from_euler("xyz", self._TARGET_POSE[3:]).as_matrix()
        diff_rot = current_rot.T @ target_rot
        diff_euler = Rotation.from_matrix(diff_rot).as_euler("xyz")
        delta = np.abs(np.hstack([current_pose[:3] - self._TARGET_POSE[:3], diff_euler]))
        return bool(np.all(delta < self._REWARD_THRESHOLD))

    # -----------------------
    # Cameras (optional)
    # -----------------------
    def init_cameras(self, name_serial_dict=None):
        """Init cameras (kept for parity; your current setup can keep this empty)."""
        if self.cap is not None:
            self.close_cameras()

        if getattr(self, "fake_env", False):
            self.cap = OrderedDict()
            return

        from franka_env.camera.video_capture import VideoCapture
        from franka_env.camera.rs_capture import RSCapture

        self.cap = OrderedDict()
        for cam_name, serial in name_serial_dict.items():
            try:
                cap = VideoCapture(RSCapture(name=cam_name, serial_number=serial))
                self.cap[cam_name] = cap
            except Exception:
                self.cap[cam_name] = None

    def close_cameras(self):
        try:
            if self.cap is None:
                return
            for cap in self.cap.values():
                if cap is not None:
                    cap.close()
        except Exception:
            pass

    def get_im(self) -> Dict[str, np.ndarray]:
        """Get images from realsense cameras (or noise in fake mode)."""
        images: dict[str, np.ndarray] = {}
        full_res_images: dict[str, np.ndarray] = {}

        if getattr(self, "fake_env", False) or getattr(self, "cap", None) is None:
            for key in self.config.REALSENSE_CAMERAS:
                fake_image = np.random.randint(0, 255, (128, 128, 3), dtype=np.uint8)
                images[key] = fake_image
                full_res_images[key] = fake_image.copy()
            if self.save_video and full_res_images:
                self.recording_frames.append(full_res_images)
            return images

        import cv2

        for key, cap in self.cap.items():
            if cap is None:
                continue
            try:
                rgb = cap.read()
                cropped_rgb = self.config.IMAGE_CROP[key](rgb) if key in self.config.IMAGE_CROP else rgb
                resized = cv2.resize(cropped_rgb, (128, 128))
                images[key] = resized[..., ::-1]
                if self.save_video:
                    full_res_images[key] = copy.deepcopy(cropped_rgb)
            except Exception:
                pass

        if self.save_video and full_res_images:
            self.recording_frames.append(full_res_images)
        return images

    # -----------------------
    # HTTP helpers
    # -----------------------
    def _send_pos_command(self, pos: np.ndarray):
        """Internal function to send pose command to the robot."""
        # DO NOT call _recover() in high-frequency control loop!
        # It will reset servo controller state and cause velocity spikes.
        # Only call _recover() explicitly in reset() or when actual error occurs.
        arr = np.asarray(pos, dtype=np.float32).reshape(-1)
        data = {"arr": arr.tolist()}
        requests.post(self.url + "pose", json=data, timeout=1.0)

    def _send_gripper_command(self, pos: float, mode="binary"):
        """Gripper is optional; if not configured, server endpoints are NO-OP."""
        if mode != "binary":
            return

        now = time.time()
        if now - self.last_gripper_act < self.gripper_sleep:
            return

        if pos <= -0.5:
            requests.post(self.url + "close_gripper", timeout=1.0)
            self.last_gripper_act = now
        elif pos >= 0.5:
            requests.post(self.url + "open_gripper", timeout=1.0)
            self.last_gripper_act = now

    def _update_currpos(self):
        """Fetch the latest robot state."""
        ps = requests.post(self.url + "getstate", timeout=2.0).json()
        self.currpos = np.asarray(ps.get("pose", np.zeros((7,))), dtype=np.float64)
        self.q = np.asarray(ps.get("q", np.zeros((6,))), dtype=np.float64)

        # Keep gripper_pose even if not present (arm-only setup)
        gp = ps.get("gripper_pos", 0.0)
        self.curr_gripper_pos = np.asarray([gp], dtype=np.float64)

    def _get_obs(self) -> dict:
        images = self.get_im()
        state_observation = {"tcp_pose": self.currpos, "gripper_pose": self.curr_gripper_pos}
        return copy.deepcopy(dict(images=images, state=state_observation))

    def close(self):
        self.close_cameras()


if __name__ == "__main__":

    env = FairinoEnv()
    print("\n[Test] Joint Reset...")
    
    input("Ready to reset?")
    env.reset(joint_reset=True)

    print("\n[Test] Sending discrete actions (Z-axis move)...")
    action_up = np.array([0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0])
    action_down = np.array([0.0, 0.0, -0.02, 0.0, 0.0, 0.0, 0.0])
    action_left = np.array([0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    action_right = np.array([-0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    input("Ready to move up?")
    env.step(action_up)
    input("Ready to move down?")
    env.step(action_down)
    input("Ready to move left?")
    env.step(action_left)
    input("Ready to move right?")
    env.step(action_right)

    env.close()
