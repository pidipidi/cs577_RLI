#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import gym
from pybulletgym.pybullet_envs.bullet.kukaGymEnv import KukaGymEnv
from stable_baselines3 import DQN
from sb3_contrib import QRDQN
import datetime

def main(episodes, renders):
  """
  Test the trained DQN model via random trials.
  """

  env = KukaGymEnv(renders=renders, isDiscrete=True)

  # Evaluate the policy
  model = QRDQN.load("kuka_model_dqn", env=env)
  
  count = 0
  success_count = 0
  
  while count < episodes:
    obs, done = env.reset(), False
    print("===================================")
    episode_rew = 0
    step_count  = 0
    while not done:
      action, _states = model.predict(obs, deterministic=True)
      obs, rew, done, _ = env.step(action)
      episode_rew += rew
      step_count += 1
    print("Episode reward {}, steps {}".format(episode_rew, step_count))

    # Check block position after finishing the episodic task.
    if env.get_block_pos()[2]>0.0:
      success_count += 1
    count += 1

  print ("Success Rate = {}".format(100.0*success_count / count))

  
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--episodes', '-n', type=int, default=50)
    parser.add_argument('--norender', '-nr', action='store_true')
    args = parser.parse_args() 
    
    main(args.episodes, not(args.norender))
