import sys
import os
import time
import numpy as np

# Add project root and examples directory to path
root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_path)
sys.path.append(os.path.join(root_path, "examples"))
sys.path.append(os.path.join(root_path, "serl_robot_infra"))

from serl_robot_infra.fairino_env.teleop.forcedimension_expert import ForceDimensionExpert
from experiments.mappings import CONFIG_MAPPING

def main():
    # 1. Initialize Robot Environment
    print("Initializing Robot Environment (Fairino)...")
    # Using 'my_test' config which we just added
    if "my_test" not in CONFIG_MAPPING:
        print("Error: 'my_test' experiment config not found in mappings.py")
        return

    config_class = CONFIG_MAPPING["my_test"]
    print(config_class)
    config = config_class()
    # Create environment (fake_env=False for real robot)
    env = config.get_environment(fake_env=False, save_video=False, classifier=False)
    
    # Use joint_reset=True to avoid using potentially unsafe Cartesian RESET_POSE from config
    obs, _ = env.reset(joint_reset=True)
    print("Robot Reset Done (Joint Reset).")

    # 2. Initialize Teleop Device
    print("Initializing Force Dimension Device...")
    # device_id=0, adjust scales as needed
    expert = ForceDimensionExpert(device_id=0, scale_pos=1.0, scale_rot=1.0)
    print("Teleop Device Initialized.")

    print("\nStarting Teleoperation Loop...")
    print("Press Ctrl+C to stop.")
    print("-" * 50)

    while True:
        # 1. Get action from Master Hand
        # action is [x, y, z, rx, ry, rz, gripper] relative delta
        action, buttons = expert.get_action()
        # print(f"Action: {action}")
        
        # 2. Step Robot Environment
        # FairinoEnv.step() takes this relative action and adds it to current pose
        print(f"Action: {action}")
        next_obs, rew, done, truncated, info = env.step(action)
        
        # Print status occasionally
        # print(f"Action: {np.round(action, 3)} | Robot Pose: {np.round(next_obs['state']['tcp_pose'], 3)}", end='\r')
        
        obs = next_obs
        
        # Optional: Exit condition based on buttons? 
        # ForceDimensionExpert returns empty list for buttons currently, can serve as extension point.

if __name__ == "__main__":
    main()
