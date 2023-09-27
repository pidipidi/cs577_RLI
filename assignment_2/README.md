# Installation
Here, you need to use the latest OpenAI Gym and RL libraries. So, please install necessary dependencies. If you want to use CUDA, before running the installation script (below), please install CUDA first. Then, run as follows:

~~~~bash
cd assignment_2
./install.sh
~~~~

This script will install a virtual environment to install dependencies. If you want different version of PyTorch, please edit the script as you want. 

# Quick Start
Let's verify if your OpenAI Gym is properly installed. This will show random grasping trials using a KUKA manipulator when you run the following command:
~~~~bash
source ./activate.sh
python test/test_pybullet_gym.py
~~~~

# Training and testing with Stable Baseline3
Please, check [Stable-Baselines3](https://stable-baselines3.readthedocs.io) library. You can grasp DQN, PPO, SAC like baseline algorithms to learn a grasping policy. Before using algorithms, you have to first complete your MDP setup for the Kuka robot and its grasping task. To do that, you need to fill out the blanks in Kuka Gym environment files below, 
~~~~bash
gedit ./pybullet-gym/pybulletgym/pybullet_envs/bullet/kukaGymEnv.py
gedit ./pybullet-gym/pybulletgym/pybullet_envs/bullet/kuka.py
~~~~

Then, you will be able to train a grasping policy using any RL algorithms. For example, you will be able to see how to import the environment and run an RL algorithm:
~~~~bash
source ./activate.sh
python test/train_kuka_grasping_baselines3.py
~~~~
It will take time until your agent gets enough rewards. After training, you can test the learned policy by loading the learned model:
~~~~bash
python test/test_kuka_grasping_baselines3.py
~~~~



# ETC
Your Gym is installed in .venv. To update pybullet_gym, you should run followings:
~~~~bash
source ./activate.sh
pip install --ignore-installed --no-deps -e ./pybullet-gym
~~~~
