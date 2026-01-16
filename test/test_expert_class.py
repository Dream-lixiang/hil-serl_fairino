import sys
import os
import time
import numpy as np

# Add project root to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from serl_robot_infra.fairino_env.teleop.forcedimension_expert import ForceDimensionExpert

def main():
    print("Initializing Force Dimension Expert...")
    # Initialize with default scaling
    # scale_pos=5.0 means 1mm device movement = 5mm action output (if interpreted directly)
    expert = ForceDimensionExpert(device_id=0, scale_pos=5.0, scale_rot=1.0)

    print("Expert Initialized.")
    print("This test verifies the ForceDimensionExpert class behavior:")
    print("1. Haptic Thread: Should feel auto-centering force.")
    print("2. Action Reading: Should print absolute position/relative rotation actions.")
    print("3. Gripper: Should show Close/Open based on gripper angle.")
    print("-" * 90)
    print(f"{'Action [x y z rx ry rz]':<55} | {'Gripper':<10} | {'Angle'}")
    print("-" * 90)

    while True:
        # Get action
        # Pass None as obs since it's not used
        action, info = expert.get_action(None)
        
        # Formatting output
        # Action: x, y, z, rx, ry, rz
        cartesian_str = ", ".join([f"{val:6.3f}" for val in action[:6]])
        
        # Gripper status from action[6]
        # -1.0 is Close, 1.0 is Open
        gripper_val = action[6]
        gripper_str = "CLOSE" if gripper_val < -0.5 else "OPEN "
        
        raw_angle = info.get("gripper_angle", "N/A")
        
        # Using carriage return to update the same line
        print(f"[{cartesian_str}] | {gripper_str} ({gripper_val:.1f}) | Angle: {raw_angle}   ", end="\r")
        
        # Sleep to simulate control loop frequency (e.g., 10Hz)
        time.sleep(0.1)

if __name__ == "__main__":
    main()