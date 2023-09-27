#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import gym
from pybulletgym.pybullet_envs.bullet.kukaGymEnv import KukaGymEnv
import torch as th
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.env_util import make_vec_env

import datetime

from typing import Any, Callable, Dict, Optional, Type, Union
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecEnv

def make_kuka_env(
    env_id: Union[str, Type[gym.Env]],
    n_envs: int = 1,
    seed: Optional[int] = None,
    start_index: int = 0,
    monitor_dir: Optional[str] = None,
    wrapper_kwargs: Optional[Dict[str, Any]] = None,
    env_kwargs: Optional[Dict[str, Any]] = None,
    vec_env_cls: Optional[Union[DummyVecEnv, SubprocVecEnv]] = None,
    vec_env_kwargs: Optional[Dict[str, Any]] = None,
    monitor_kwargs: Optional[Dict[str, Any]] = None,
) -> VecEnv:
    """
    Create a wrapped, monitored VecEnv for Atari.
    It is a wrapper around ``make_vec_env`` that includes common preprocessing for Atari games.

    :param env_id: the environment ID or the environment class
    :param n_envs: the number of environments you wish to have in parallel
    :param seed: the initial seed for the random number generator
    :param start_index: start rank index
    :param monitor_dir: Path to a folder where the monitor files will be saved.
        If None, no file will be written, however, the env will still be wrapped
        in a Monitor wrapper to provide additional information about training.
    :param wrapper_kwargs: Optional keyword argument to pass to the ``AtariWrapper``
    :param env_kwargs: Optional keyword argument to pass to the env constructor
    :param vec_env_cls: A custom ``VecEnv`` class constructor. Default: None.
    :param vec_env_kwargs: Keyword arguments to pass to the ``VecEnv`` class constructor.
    :param monitor_kwargs: Keyword arguments to pass to the ``Monitor`` class constructor.
    :return: The wrapped environment
    """
    if wrapper_kwargs is None:
        wrapper_kwargs = {}

    def kuka_wrapper(env: gym.Env) -> gym.Env:
        env = KukaGymEnv(renders=False, isDiscrete=True)
        return env

    return make_vec_env(
        env_id,
        n_envs=n_envs,
        seed=seed,
        start_index=start_index,
        monitor_dir=monitor_dir,
        wrapper_class=kuka_wrapper,
        env_kwargs=env_kwargs,
        vec_env_cls=vec_env_cls,
        vec_env_kwargs=vec_env_kwargs,
        monitor_kwargs=monitor_kwargs,
    )


def main(renders):
  """
  Train a deep q-learning model in Stable Baselines3
  """
  os.environ['OPENAI_LOGDIR'] = "./logs/ppo"

  # --------------------------------------------------------------
  # Place your code here! 
  # --------------------------------------------------------------
  ## env = make_vec_env("KukaBulletEnv-v0", n_envs=2, seed=0)
  env = make_kuka_env("KukaBulletEnv-v0", n_envs=8, seed=0)
  eval_env = KukaGymEnv(renders=False, isDiscrete=True)
  policy_kwargs = dict(activation_fn=th.nn.Tanh,
                         net_arch=[32,32])
  model = PPO("MlpPolicy", env,
                learning_rate=0.0001,
                n_steps=4096, #256,
                batch_size=1024, #512
                gamma=0.8,
                ## clip_range=0,
                policy_kwargs=policy_kwargs, 
                seed=0,
                device='cuda',
                verbose=1,
                tensorboard_log="./ppo_kuka_tensorboard/")  
  model.learn(total_timesteps=4000000, log_interval=10, eval_env=eval_env, eval_freq=10000, n_eval_episodes=10)  
  model.save("kuka_model_ppo")
  print("Saving model to kuka_model_ppo")
  # --------------------------------------------------------------
  
  # Evaluate the policy
  mean_reward, std_reward = evaluate_policy(model, model.get_env(), n_eval_episodes=10, return_episode_rewards=True)
  print(mean_reward)
  print(std_reward)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--norender', '-nr', action='store_true')
    args = parser.parse_args() 
    
    main(not(args.norender))
