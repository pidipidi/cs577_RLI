#!/usr/bin/env python
"""
Copyright 2020 Daehyung Park

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
import gym
from gym import error, spaces, utils
from gym.utils import seeding

import time, copy
import random
import math
import numpy as np
import scipy.spatial
from sklearn.neighbors import BallTree

import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib
matplotlib.use("TkAgg")


FPS=50

class ReachingEnv2D(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : FPS
        }
    def __init__(self, **kwargs):
        self.robot_size = 0.5
        self.goal_size  = 1.

        self.seed(0)
        self.states = None
        self.first_render = False

    @property
    def action_space(self):
        return spaces.Box( -5., 5., (2,), dtype=np.float64)

    @property
    def observation_space(self):
        return spaces.Box( 0., 20., (2,), dtype=np.float64)

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]


    def step(self, action):
        ''' return next observation, reward, finished, success '''
        action = np.clip(action, self.action_space.low, self.action_space.high)
        state  = self.state + action

        info = {}
        if self.isValid(state, check_collision=True):
            # succeeded state        
            info['success'] = True
            reward = self.get_reward(state) 
        else:
            # failed state        
            info['success'] = False
            #reward = self.get_reward(self.state) 

            # by cbs
            return (self.state, -100., False, info)
            #return (self.state, reward, False, info)
            
        self.state = state

        # terminal state        
        done = self.isGoal(self.state)
        if done:
            self.state = self.goal_state

        return (self.state, reward, done, info)
            

    def reset(self):
        self.state = self.start_state
        return self.start_state

    
    def isGoal(self, state):
        """Check goal"""
        if np.linalg.norm(state-self.goal_state) < self.goal_size:
            return True
        return False
    
    def isValid(self, state, check_collision=False):
        """Check validity of current state"""
        # work space
        if any(state[i] < self.observation_space.low[i] for i in range(self.observation_space.shape[0])):
            return False
        if any(state[i] > self.observation_space.high[i] for i in range(self.observation_space.shape[0])):
            return False

        # collision 
        if self.tree is not None and check_collision:
            if len(np.shape(state))==1: state=state[np.newaxis,:]
            count = self.tree.query_radius(state, self.robot_size,
                                               count_only=True)
            if count>0: return False
        return True

    def render(self, mode="human"):

        if self.first_render is False:
            self.fig, self.ax = plt.subplots()
            self.ax.set_aspect('equal')
            
            if len(self.objects)>0:
                plt.plot(self.objects[:,0], self.objects[:,1], 'ko')
            plt.plot(self.start_state[0], self.start_state[1], 'rx')
            plt.plot(self.goal_state[0], self.goal_state[1], 'r^')

            plt.xlim(self.observation_space.low[0], self.observation_space.high[0] )
            plt.ylim(self.observation_space.low[1], self.observation_space.high[1] )
            self.first_render = True
            
        # plot current point?
        plt.plot(self.state[0], self.state[1], '.b')
        plt.pause(0.001)

    def render_value_map(self, Q, n):
        '''
            render state or action value grid map.
            V[s] = max(Q[s,a])

            Q--> numpy array
            [[a1, a2, a3],
            [a1, a2, a3],
            ...]
        '''
        if len(np.shape(Q))>1:
            V = np.amax(Q, axis=1) 
            V = V.reshape((n,n)).T
            plt.imshow(V, cmap='jet', interpolation='nearest')
        else:
            V = Q.reshape((n,n)).T
            plt.imshow(V, cmap='jet', interpolation='nearest')
            plt.gca().invert_yaxis()

        plt.show()


    # --------------------- Get/Set -----------------------
    def get_start_state(self):
        return self.start_state
    def get_goal_state(self):
        return self.goal_state
    def get_objects(self):
        return self.objects
    def get_reward(self, state):
        return self.reward_fn(state)
    ## def get_rewards(self, states):
    ##     return self.reward_fn(states)
    def get_states(self):
        return self.states    
        
    def set_start_state(self, state):
        self.start_state = np.array(copy.deepcopy(state))
    def set_goal_size(self, goal_size):
        self.goal_size = goal_size
    def set_goal_state(self, state):
        self.goal_state = np.array(copy.deepcopy(state))
        self.reward_fn = make_default_reward(self.goal_state)        
    def set_objects(self, objs):
        self.objects = np.array(objs)
        self.tree = BallTree(self.objects)
    ## def set_rewards(self, rewards):
    ##     self.rewards = rewards
    def set_reward(self, reward_fn):
        self.reward_fn = reward_fn
    def set_robot_size(self, robot_size):
        self.robot_size = robot_size
    def set_states(self, states):
        self.states = states


def make_default_reward(goal_state):
    '''
    Make a reward that returns a high reward at the goal location
    '''
    if type(goal_state) is list: goal_state = np.array(goal_state)
    
    def reward_fn(state):
        if type(state) is list: state = np.array(state)
        
        if len(np.shape(state))>=2:
            r = np.zeros(len(state))
            idx = np.argmin(np.linalg.norm(state-goal_state, axis=1))
            r[idx]=1.
            ## r = np.exp(- np.linalg.norm(state-goal_state, axis=1))
        else:
            if np.linalg.norm(state-goal_state)<1e-5: r = 1.
            else: r=0.
            ## r = np.exp(- np.linalg.norm(state-goal_state))
        return r

    return reward_fn
    
