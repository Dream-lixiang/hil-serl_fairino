import os
from tqdm import tqdm
import numpy as np
import copy
import pickle as pkl
import datetime
from absl import app, flags
import time

from experiments.mappings import CONFIG_MAPPING
from serl_robot_infra.fairino_env.teleop import ForceDimensionExpert

FLAGS = flags.FLAGS
flags.DEFINE_string("exp_name", None, "Name of experiment corresponding to folder.")
flags.DEFINE_integer("successes_needed", 20, "Number of successful demos to collect.")
flags.DEFINE_integer("device_id", 0, "Force Dimension device ID.")

def main(_):
    if FLAGS.exp_name not in CONFIG_MAPPING:
        print(f"Experiment {FLAGS.exp_name} not found in mappings.")
        return

    config = CONFIG_MAPPING[FLAGS.exp_name]()
    # Enable classifier for success detection if needed
    env = config.get_environment(fake_env=False, save_video=False, classifier=True)
    
    # Initialize the expert (Force Dimension master hand)
    print(f"Initializing Force Dimension device {FLAGS.device_id}...")
    expert = ForceDimensionExpert(device_id=FLAGS.device_id)
    
    obs, info = env.reset()
    print("Reset done. Ready to record.")
    
    transitions = []
    success_count = 0
    success_needed = FLAGS.successes_needed
    pbar = tqdm(total=success_needed)
    trajectory = []
    returns = 0
    
    while success_count < success_needed:
        # Get action from the master hand
        actions, _ = expert.get_action(obs)
        
        # Step the environment
        next_obs, rew, done, truncated, info = env.step(actions)
        returns += rew
        
        # Handle intervention if any (though expert is driving)
        if "intervene_action" in info:
            actions = info["intervene_action"]
            
        transition = copy.deepcopy(
            dict(
                observations=obs,
                actions=actions,
                next_observations=next_obs,
                rewards=rew,
                masks=1.0 - done,
                dones=done,
                infos=info,
            )
        )
        trajectory.append(transition)
        
        pbar.set_description(f"Return: {returns}")

        obs = next_obs
        if done:
            if info.get("succeed", False): # Use get() just in case
                for transition in trajectory:
                    transitions.append(copy.deepcopy(transition))
                success_count += 1
                pbar.update(1)
            else:
                print("Episode failed/done without success.")
                
            trajectory = []
            returns = 0
            obs, info = env.reset()
            # Expert state might need reset? Usually relative, so maybe fine.
            # But if using relative pos from prev step, no reset needed.
            
    expert.close()
    
    if not os.path.exists("./demo_data"):
        os.makedirs("./demo_data")
    uuid = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    file_name = f"./demo_data/{FLAGS.exp_name}_{success_needed}_demos_{uuid}.pkl"
    with open(file_name, "wb") as f:
        pkl.dump(transitions, f)
        print(f"saved {success_needed} demos to {file_name}")

if __name__ == "__main__":
    app.run(main)
