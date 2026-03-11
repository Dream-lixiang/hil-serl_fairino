import sys
import os
import time
import threading
import ctypes
import numpy as np

# Ensure repo root is first on sys.path so local modules override site-packages
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# If a pip-installed forcedimension_core was imported earlier, drop it.
for name in list(sys.modules.keys()):
    if name == "forcedimension_core" or name.startswith("forcedimension_core."):
        del sys.modules[name]

# Import necessary modules
# Ensure forcedimension_core and realrobot are in your python path
import forcedimension_core.dhd as dhd
import forcedimension_core.drd as drd
from scipy.spatial.transform import Rotation as R
from serl_robot_infra.fairino_servers.Fairino_Arm.fairino import Robot

class RealRobotEnv:
    def __init__(self, ip, initial_joint_pos, robot_lock, print_lock):
        self.robot = Robot.RPC(ip)
        self.robot_lock = robot_lock
        self.print_lock = print_lock
        self.initial_joint_pos = initial_joint_pos
        self.reset()

    def reset(self):
        with self.robot_lock:
            # Check if ResetAllError exists, otherwise skip or handle differently
            try:
                self.robot.ResetAllError()
            except AttributeError:
                pass 
                
            tool, user = 1, 0  # Tool and User coordinate system IDs
            self.robot.MoveJ(self.initial_joint_pos, tool, user, vel=10)
            
            # Wait for motion done (simplified check)
            # Assuming GetRobotMotionDone returns [err, state] where state 1 is done
            while True:
                res = self.robot.GetRobotMotionDone()
                if res[1] == 1:
                    break
                time.sleep(0.1)
            time.sleep(1)
        with self.print_lock:
            print(f"Robot ({self.robot}) reset complete.")

def mat2euler(matrix):
    """
    Convert rotation matrix to euler angles (xyz).
    Using scipy as 'diffusion_policy.env.surrol.rotations' might not be available.
    """
    r = R.from_matrix(matrix)
    return r.as_euler('xyz')

def control_robot(env, mode, device_id, flag_exit, locks):
    pos_start = np.zeros(3)
    matrix_start = np.eye(3)
    
    pos = np.zeros(3)
    matrix = np.eye(3)
    
    robot_ctrl = np.zeros(6)

    # Initialize start state
    dhd.getPositionAndOrientationFrame(pos_start, matrix_start, device_id)
    
    print(f"Control loop started for device {device_id}")

    try:
        while not flag_exit[0]:
            # 1. Read Device State
            # Simulating loop delay (e.g. 50Hz) to match control frequency
            dhd.os_independent.sleep(0.02) 
            
            dhd.getPositionAndOrientationFrame(pos, matrix, device_id)

            # 2. Set Haptic Forces (Gravity Compensation / Zero Force)
            if dhd.setForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, device_id) < 0:
                with locks['print']:
                    print(f"Failed to set forces for device {device_id}")
                break

            # 3. Generate Robot Action
            # Calculate Delta Position (Scaled)
            delta_pos = (pos - pos_start) * 2000.0 # Scaling factor from example
            delta_pos = np.round(delta_pos, 2)
            
            # Calculate Delta Orientation
            # R_diff = R_curr * R_start^-1
            mat_diff = np.dot(matrix, np.linalg.inv(matrix_start))
            delta_euler = mat2euler(mat_diff) * 20.0 # Scaling factor
            delta_euler = np.round(delta_euler, 3)

            # Map to Robot Control Frame
            # Mapping: x->x, y->-y, z->-z (from example)
            robot_ctrl[:] = [
                delta_pos[1],
                delta_pos[0],
                -delta_pos[2],
                delta_euler[1],
                delta_euler[0],
                -delta_euler[2]
            ]

            # 4. Send Command
            with locks['robot']:
                # ServoCart: mode 2 is usually incremental cartesian
                error = env.robot.ServoCart(mode, robot_ctrl, vel=3)
            
            if error != 0:
                with locks['print']:
                    # Rate limit error printing if needed
                    print("ServoCart error:", error)

            # Update 'start' pose for next incremental step?
            # If control is absolute relative to initial grasp, we don't update pos_start.
            # If control is velocity (incremental), we should update pos_start to current.
            # The example code seems to use:
            #   dhd.getPosition... (pos_start)
            #   sleep
            #   dhd.getPosition... (pos)
            #   delta = pos - pos_start
            #   ...
            #   loop continues
            #
            # BUT: In the provided example loop, `pos_start` is NEVER updated inside the loop!
            # This implies it calculates delta relative to the position *at the very beginning of the loop*?
            # Wait, the example code reads pos_start INSIDE the loop:
            #   while True:
            #      dhd.getPosition...(pos_start)  <-- Reads current pos as start
            #      sleep(0.02)
            #      dhd.getPosition...(pos)        <-- Reads pos 20ms later
            #
            # So it IS a velocity control (delta over 20ms).
            # Let's match that logic.
            
            # Update start state for next iteration (sliding window)
            pos_start[:] = pos[:]
            matrix_start[:] = matrix[:]

    except Exception as e:
        print(f"Control loop exception: {e}")
    finally:
        # Stop Servo Mode
        with locks['robot']:
            env.robot.ServoMoveEnd()
        print(f"Control loop ended for device {device_id}")

def initialize_device(print_lock):
    device_count = dhd.getDeviceCount()
    if device_count < 1:
        with print_lock:
            print("No Force Dimension devices found!")
        sys.exit(1)
    
    device_id = dhd.openID(0)
    with print_lock:
        print(f"Found device 0: ID {device_id}")
    
    if drd.openID(0) < 0:
        print("Failed to open DRD")
        sys.exit(1)
        
    if not drd.isInitialized(0):
        print("Initializing device...")
        if drd.autoInit(0) < 0:
            print("Failed to auto-init")
            sys.exit(1)
            
    if drd.start(0) < 0:
        print("Failed to start DRD")
        sys.exit(1)
        
    # Stop regulation (enable gravity comp only)
    drd.stop(True, 0)
    
    return device_id

def main():
    print_lock = threading.Lock()
    robot_lock = threading.Lock()

    # 1. Initialize Device
    device_id = initialize_device(print_lock)

    # 2. Initialize Robot
    # IP and Home Position (Adjust IP as needed!)
    ROBOT_IP = '192.168.58.6' 
    HOME_POS = [-90.0, -90.0, -90.0, -90.0, 90.0, 0.0] # Example home pos
    
    print(f"Connecting to robot at {ROBOT_IP}...")
    try:
        env = RealRobotEnv(ROBOT_IP, HOME_POS, robot_lock, print_lock)
    except Exception as e:
        print(f"Failed to connect to robot: {e}")
        return

    # 3. Start Servo Mode
    mode = 2 # Incremental Cartesian Tool Frame?
    with robot_lock:
        ret = env.robot.ServoMoveStart()
        if ret != 0:
            print(f"Failed to start servo mode: error {ret}")
            return
    print("Servo mode started.")

    print("\n" + "="*50)
    print(" Control Started. Press 'q' + Enter in terminal to Exit.")
    print(" (Or Ctrl+C)")
    print("="*50 + "\n")

    flag_exit = [False]
    
    # 4. Start Control Thread
    t = threading.Thread(
        target=control_robot, 
        args=(env, mode, device_id, flag_exit, {'robot': robot_lock, 'print': print_lock})
    )
    t.start()

    try:
        while t.is_alive():
            user_input = input()
            if user_input.strip() == 'q':
                flag_exit[0] = True
                break
    except KeyboardInterrupt:
        flag_exit[0] = True
    
    t.join()
    
    # 5. Cleanup
    print("Cleaning up...")
    drd.close(device_id)
    dhd.close(device_id)
    env.reset() # Reset robot to home
    print("Done.")

if __name__ == "__main__":
    main()
