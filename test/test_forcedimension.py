import sys
import os
import time
import numpy as np

# Add the project root to sys.path to allow importing local forcedimension_core and serl_robot_infra
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    # 插到最前面，覆盖掉可能存在的 pip 版 forcedimension_core
    sys.path.insert(0, project_root)

from serl_robot_infra.fairino_env.teleop.forcedimension_expert import ForceDimensionExpert

def main():
    print("Initializing Force Dimension device...")
    # Initialize with device_id=0 (default)
    expert = ForceDimensionExpert(device_id=0, scale_pos=1.0, scale_rot=1.0)

    print("Device initialized successfully.")
    print("Reading inputs... Press Ctrl+C to exit.")
    print("-" * 50)
    print(f"{'Action (XYZ RPY Gripper)':<50}")
    print("-" * 50)

    while True:
        # Pass None as obs since ForceDimensionExpert doesn't use it
        action, buttons = expert.get_action(None)
        
        # Formatting output for readability
        # action is [x, y, z, rx, ry, rz, gripper]
        action_str = ", ".join([f"{val:6.3f}" for val in action])
        
        # Also print raw rotation from the device to debug mapping if needed
        # We can't easily access internal state of expert from here without modification
        # But action already reflects the rotated mapping.
        
        print(f"Action: [{action_str}]", end="\r")
        
        time.sleep(0.05)

if __name__ == "__main__":
    main()
