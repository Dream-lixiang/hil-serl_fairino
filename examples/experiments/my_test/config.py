import numpy as np
from serl_robot_infra.fairino_env.envs.fairino_env import DefaultEnvConfig, FairinoEnv

class MyTestEnvConfig(DefaultEnvConfig):
    # Set your specific configuration here
    # Example RESET_POSE: [x, y, z, rx, ry, rz] (in meters and radians)
    # Adjust these values to a safe starting position for your robot
    RESET_POSE = np.array([-90.0, -90.0, -90.0, -90.0, 90.0, 0.0]) 
    
    ACTION_SCALE = np.array([0.5, 0.5, 1.0]) # Adjust scaling if needed
    
    # Define camera config if you have cameras, otherwise empty dict is fine for basic movement
    REALSENSE_CAMERAS = {} 
    
class TrainConfig:
    # Minimal TrainConfig to satisfy the mapping interface
    def get_environment(self, fake_env=False, save_video=False, classifier=False):
        env = FairinoEnv(
            fake_env=fake_env,
            save_video=save_video,
            config=MyTestEnvConfig(),
            # hz=50  # Increase to 50Hz for smoother control
        )
        return env
