#!/usr/bin/env python
import numpy as np
import gym
import rli_gym
from vi import valueIterAgent
from sklearn.neighbors import BallTree
import itertools

def generate_obstacles(xlim, ylim):
    """ 
    A function to generate obstacles

    Parameters
    ----------
    xlim : list
        [lower limit, upper limit] of x coordinate
    ylim : list
        [lower limit, upper limit] of y coordinate

    Returns
    -------
    obstacles : list
        a list of obstacle coordinates.
    obstacle_tree : BallTree 
        a BallTree object from Scikit-Learn. The tree includes
        the coordinates of obstacles.
    """
    obstacles = [[3,16],[3,17],
                     [4,8],[4,16],[4,17],
                     [5,8],[5,11],[5,12],[5,13],[5,16],[5,17],
                     [6,6],[6,7],[6,8],[6,11],[6,12],[6,13],[6,16],[6,17],
                     [7,6],[7,7],[7,8],[7,12],[7,13],
                     [8,6],[8,7],[8,8],[8,12],[8,13],
                     [9,12],[9,13],
                     [10,16],[10,17],
                     [12,7],[12,8],[12,9],
                     [13,4],[13,5],[13,7],[13,8],[13,9],[13,13],[13,16],[13,17],
                     [14,13],[15,13],[15,17],[15,18],
                     [16,8],[16,9],[16,10],[16,13],[16,17],[16,18],
                     [17,8],[17,9],[17,10],[17,13],[18,8],[18,9],[18,10] ]

    obstacle_tree = BallTree(np.array(obstacles))
        
    return obstacles, obstacle_tree


def make_reward(env, goal_state):
    '''
    Make a reward that returns a high reward at the goal location
    '''
    if type(goal_state) is list: goal_state = np.array(goal_state)

    def reward_fn(state):
        if type(state) is list: state = np.array(state)
        if env.isValid(state, check_collision=True) is False: return -1
        if state[0]==goal_state[0] and state[1]==goal_state[1]: return 1
        else:                 return 0
    return reward_fn


if __name__ == '__main__':

    # Initialize variables
    start = [3,18]
    goal  = [18,3]
    xlim  = [0,20]
    ylim  = [0,20]
    obstacles, obstacle_tree = generate_obstacles([0,20],[0,20])
    grid_size  = 1.0
    robot_size = 0.5

    # initialize openai gym
    env = gym.make("reaching-v0")
    env.set_start_state(start)
    env.set_goal_size(1.0)
    env.set_goal_state(goal)
    env.set_objects(obstacles)
    env.set_robot_size(robot_size)    
    env.set_reward(make_reward(env, goal))
    env.reset()

    # run your algorithm
    vi = valueIterAgent(env)
    vi.learn()
    vi.save()
    #vi.load()
    vi.test()
