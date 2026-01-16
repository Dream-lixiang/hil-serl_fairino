import threading
import ctypes
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import forcedimension_core.dhd as dhd
import forcedimension_core.drd as drd

class ForceDimensionExpert:
    def __init__(self, device_id=0, scale_pos=0.5, scale_rot=1.0):
        self.device_id = device_id
        self.scale_pos = scale_pos
        self.scale_rot = scale_rot
        
        # Initialize device
        if dhd.getDeviceCount() < 1:
            raise RuntimeError("No Force Dimension devices found")
            
        self.id = dhd.openID(device_id)
        if self.id < 0:
             raise RuntimeError(f"Could not open device {device_id}")
             
        # Initialize DRD (Robotic SDK)
        if drd.openID(device_id) < 0:
             print(f"Warning: drd.openID failed for {device_id}")
        
        if not drd.isInitialized(device_id):
             if drd.autoInit(device_id) < 0:
                 print(f"Warning: drd.autoInit failed for {device_id}")
        
        if drd.start(device_id) < 0:
            print(f"Warning: drd.start failed for {device_id}")

        # Stop regulation to allow free movement with gravity compensation
        drd.stop(True, device_id)
        
        # Initial state variables placeholder
        self.pos_prev_served = np.zeros(3)
        self.mat_prev_served = np.eye(3)
        
        # Shared state for thread communication
        self.running = True
        self.latest_pos = np.zeros(3)
        self.latest_rot = np.eye(3)
        self.latest_vel = np.zeros(3) # 新增：用于存储速度
        self.latest_buttons = 0
        self.latest_gripper_angle = 0
        self.lock = threading.Lock()
        
        # Start Haptic Thread (Auto-centering logic)
        print("Force Dimension: Starting haptic loop...")
        self.thread = threading.Thread(target=self._haptic_loop, daemon=True)
        self.thread.start()

        # --- 关键修改：等待归中稳定 ---
        print("Force Dimension: Auto-centering... (Waiting 2s)")
        time.sleep(2.0) 

        # --- 关键修改：重置上一帧状态 ---
        # 归中完成后，重新读取当前位置作为“上一帧”，防止第一帧Action出现巨大的跳变
        with self.lock:
            # 更新 prev 为 归中后 的状态
            self.pos_prev_served = self.latest_pos.copy()
            self.mat_prev_served = self.latest_rot.copy()
        
        print("Force Dimension: Ready.")
        
    def _haptic_loop(self):
        """
        Runs at high frequency (~1kHz).
        Includes Spring (P), Integral (I), and Damping (D) to prevent oscillation and correct gravity bias.
        """
        # Constants
        K_spring = 200.0   # N/m (P gain)
        K_damping = 10.0   # N/(m/s) (D gain) - 增加阻尼防止震荡
        K_integral = 80.0  # N/(m*s) (I gain) - 积分项，用于消除重力导致的稳态误差
        K_torsion = 5.0    # Nm/rad
        
        pos = np.zeros(3)
        rot = np.eye(3)
        vel = np.zeros(3)  # Velocity buffer
        integral_error = np.zeros(3) # 积分误差累积
        gripper_ptr = ctypes.pointer(ctypes.c_double(0.0))
        
        loop_dt = 0.001 # 假设 1kHz 循环
        
        while self.running:
            # 1. Read Device State
            if dhd.getPositionAndOrientationFrame(pos, rot, self.id) < 0:
                time.sleep(loop_dt)
                continue
            
            # 读取线速度用于阻尼计算
            dhd.getLinearVelocity(vel, self.id)
            
            dhd.getGripperAngleDeg(gripper_ptr, self.id)
            gripper_angle = int(gripper_ptr.contents.value)
            btn = dhd.getButton(0, self.id)
                
            # 2. Update Shared State
            with self.lock:
                self.latest_pos[:] = pos
                self.latest_rot[:] = rot
                self.latest_vel[:] = vel
                self.latest_buttons = btn
                self.latest_gripper_angle = gripper_angle
            
            # 3. Calculate Forces (PID Control)
            # 积分误差累积 (I项)
            # 只有当位置比较接近中心时才积分，防止大幅度运动时积分过大
            if np.linalg.norm(pos) < 0.05: 
                integral_error += pos * loop_dt
            else:
                # 距离太远时（比如人为拖动），暂时冻结积分或缓慢衰减，防止松手后反弹过猛
                integral_error *= 0.99
            
            # Anti-windup: 限制积分项产生的力不超过一定范围 (例如 3N，足以抵抗重力但不会伤人)
            max_integral_force = 3.0
            max_integral_val = max_integral_force / K_integral
            integral_error = np.clip(integral_error, -max_integral_val, max_integral_val)

            # F = -K_p * x - K_i * sum(x) - K_d * v
            force = -K_spring * pos - K_integral * integral_error - K_damping * vel
            
            # Clamp Force (Safety)
            force_mag = np.linalg.norm(force)
            if force_mag > 15.0: #稍微放宽一点上限给阻尼发挥作用
                force = force * (15.0 / force_mag)
                
            # Orientation Spring (Torque)
            r = R.from_matrix(rot)
            rot_vec = r.as_rotvec()
            torque = -K_torsion * rot_vec
            
            # Clamp Torque (Safety)
            torque_mag = np.linalg.norm(torque)
            if torque_mag > 0.5:
                torque = torque * (0.5 / torque_mag)
                
            # 4. Apply Forces to Device
            dhd.setForceAndTorqueAndGripperForce(
                force[0], force[1], force[2], 
                torque[0], torque[1], torque[2], 
                0.0, 
                self.id
            )
            
            # 1kHz loop rate is handled by DHD usually, but sleep helps if DHD is non-blocking
            time.sleep(0.001)
            
    def get_action(self, obs=None):
        """
        Reads the latest device state and returns the action.
        """
        with self.lock:
            pos_curr = self.latest_pos.copy()
            mat_curr = self.latest_rot.copy()
            gripper_angle = self.latest_gripper_angle
            
        # --- POSITION MAPPING (ABSOLUTE) ---
        raw_pos_input = pos_curr * self.scale_pos
        
        # --- ROTATION MAPPING (RELATIVE) ---
        R_curr = mat_curr
        R_prev = self.mat_prev_served
        # Calculate Delta Rotation: R_diff = R_curr @ R_prev.T
        R_diff = R_curr @ np.linalg.inv(R_prev)
        delta_euler = R.from_matrix(R_diff).as_euler('xyz') * self.scale_rot
        
        # Deadzones
        if np.linalg.norm(raw_pos_input) < 1e-2:
            raw_pos_input[:] = 0.0
        if np.linalg.norm(delta_euler) < 1e-3:
            delta_euler[:] = 0.0

        action = np.zeros(7)
        
        # Mapping (Position - Absolute)
        # Assuming FD mapping: x(right), y(up), z(back/front)
        # Target mapping: x(forward), y(left), z(up) -> Adjust as per your robot frame
        action[0] = -raw_pos_input[1] # -Y -> X
        action[1] = raw_pos_input[0]  # X  -> Y
        action[2] = raw_pos_input[2]  # Z  -> Z
        
        # Mapping (Rotation - Relative)
        action[3] = -delta_euler[1]
        action[4] = delta_euler[0]
        action[5] = delta_euler[2]
        
        # Gripper
        if gripper_angle < 16:
            action[6] = -1.0 
        else:
            action[6] = 1.0
        
        # Update previous served state
        self.pos_prev_served = pos_curr
        self.mat_prev_served = mat_curr
        
        return action, {"gripper_angle": gripper_angle}

    def close(self):
        self.running = False
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join()
        drd.close(self.id)
        dhd.close(self.id)