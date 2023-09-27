# -*- coding: utf-8 -*-
import numpy as np
import pickle
import copy
import itertools


class valueIterAgent:
    def __init__(self, env, gamma=0.9):
        """
        A function to select an action index given a state

        Parameters
        ----------
        env object: Gym Environment
        gamma float: discount factor
        """

        self.env         = env
        self.gamma       = gamma
        self.grid_limits = np.array([env.observation_space.low, env.observation_space.high]).astype(int) 
        self.resolution  = 1 #resolution
        self.grid_dim    = np.round((self.grid_limits[1]-self.grid_limits[0])/self.resolution+1).astype(int)
        self.actions    = np.array([[-1,0], [0,-1], [1,0], [0,1]])
        self.action_dim = len(self.actions)
        
        self.policy    = None
        self.values    = None

        x = range(self.grid_limits[0][0], self.grid_limits[1][0]+1)
        y = range(self.grid_limits[0][1], self.grid_limits[1][1]+1)
        self.states = np.array(list(itertools.product(x,y)))
        ## self.T      = create_transition_matrix(len(env.observation_space.low), self.action_dim)
        self.observation_dim = len(self.states)


    def get_grid_index(self, state):
        """ 
        Compute a grid index given an input state

        Parameters
        ----------
        state list/array: agent's status
        """
        ids = np.round((state-self.grid_limits[0])/self.resolution).astype(int)

        if len(state)==2:
            return ids[0]*self.grid_dim[1]+ids[1]

        return NotImplemented
        

    def choose_action(self, state):
        """
        Return an action index given a state

        Parameters
        ----------
        state list/array: agent's status
        """
        #------------------------------------------------------------
        # Place your code here
        # -----------------------------------------------------------
        return 
        # -----------------------------------------------------------
        

    def solve_mdp(self, error=1e-10, **kwargs):
        """
        Compute values via value iterations

        Parameters
        ----------
        error float: a convergence check threshold
        """
        
        values = np.zeros(len(self.states))
        nxt_values = np.zeros(len(self.states))
        
        max_cnt = kwargs.get('max_cnt', None)

        # update each state
        diff  = float("inf")
        count = 0
        while True:
            count += 1
            
            #------------------------------------------------------------
            # Place your code here
            # -----------------------------------------------------------
            #for i, state in enumerate(self.states):
                # ...

                           
            #if diff <= error:
                # ...
                #break
                
            # -----------------------------------------------------------
            
            values = copy.copy(nxt_values)            
            print ("Value error: {}".format(diff))
        print ("error converged to {} after {} iter".format(diff, count))
 
        return values


    def learn(self, error=1e-9):
        """ 
        Run the value iteration to obtain a policy
        
        Parameters
        ----------
        error float: a convergence check threshold
        """
        
        values = self.solve_mdp(error)                

        # generate stochastic policy
        policy = np.zeros([self.observation_dim, self.action_dim])            
        #------------------------------------------------------------
        # Place your code here
        # -----------------------------------------------------------
        #for s in range(self.observation_dim):
        #    for a in range(self.action_dim):
                # ...


                
        # -----------------------------------------------------------
        
        self.policy = policy
        self.values = values
        return self.policy, self.values

    def get_policy(self):
        """Get the current policy"""
        return self.policy

    def test(self):
        """test"""
        state = self.env.reset()
        path  = [self.env.get]

        self.env.render_value_map(self.values, self.grid_dim[0])
        while True:
            self.env.render()

            action = self.choose_action(state)
            next_state, reward, done, info = self.env.step(self.actions[action])
            state = next_state

            if done:
                self.env.close()
                break
        return path
    
    def save(self, filename='vi.pkl'):
        """Save the obtained state values"""
        d = {'values': self.values}
        pickle.dump(d, open(filename, "wb"))

    def load(self, filename='vi.pkl'):
        """Load the stored state values"""
        d = pickle.load(open(filename, "rb"))
        self.values = d['values']
    
