#!/usr/bin/env python3
import numpy as np
import min_jerk as mj
from dmp import *

def min_jerk_trajs(dims=1, n_samples=10, freq=100, duration=1., add_noise=False):
    """
    Generate a set of minimum-jerk trajectories [n_samples x dims x n_length].

    dims   : the dimension of a sample trajectory
    n_samples: the number of trajectories
    freq     : frequency
    duration : time length of trajectories
    add_noise: add gaussian noise
    """
    samples = []
    for i in range(n_samples):

        start = np.zeros(dims)
        goal  = np.ones(dims)
        
        if add_noise:            
            start += np.random.normal(0,0.04,dims)
            goal  += np.random.normal(0,0.04,dims)
       
        _, trj, trj_vel, trj_acc, trj_jerk = mj.min_jerk(start, goal, duration, freq)
        samples.append(trj.T.tolist())

    return np.array(samples)


def problem_1a():
    """
    Train a DMP with a min-jerk traj and reproduce the same traj.
    """    
    n_samples = 1
    dims      = 1
    bfs       = 20
    tau       = 1.
    
    # Data generation
    trajs_demo = min_jerk_trajs(dims=dims, n_samples=n_samples)

    # Learn via DMP original/improved
    dmp = DMPs_discrete(dims=dims, bfs=bfs, tau=tau)
    dmp.learn(trajs_demo)

    # ReProduce a trajectory
    traj, _, _ = dmp.plan()

    # Reproduction w/ visualization
    dmp.plot_traj(trajs_demo, np.expand_dims(traj, axis=0))
    dmp.plot_basis()


def problem_1b():
    """
    Train a DMP with a min-jerk traj and reproduce it with different goals.
    """    
    n_samples = 1
    dims      = 1
    bfs       = 20
    tau       = 1.
    
    # Data generation
    trajs_demo = min_jerk_trajs(dims=dims, n_samples=n_samples)

    # Learn via DMP original/improved
    dmp = DMPs_discrete(dims=dims, bfs=bfs, tau=tau)
    dmp.learn(trajs_demo)

    # ReProduce a trajectory
    y0    = None
    trajs = []

    #------------------------------------------------------------
    # Place your code here
    for i in range(10):
        goal = np.array([1.0*((i-5)/2.0),])
        traj, _, _ = dmp.plan(y0=y0, goal=goal)
        trajs.append(traj)
    #------------------------------------------------------------

    # Reproduction w/ visualization
    dmp.plot_traj(trajs_demo, np.array(trajs))

    
def problem_1c():
    """
    Train DMPs with multiple min-jerk trajectories and reproduce 
    the demo with the same goal.
    """
    n_samples = 10
    dims      = 2
    bfs       = 20
    tau       = 1.
    
    # Data generation
    trajs_demo = min_jerk_trajs(dims=dims, n_samples=n_samples, add_noise=True)

    # Learn via DMP original/improved
    dmp = DMPs_discrete(dims=dims, bfs=bfs, tau=tau)
    dmp.learn(trajs_demo)

    # ReProduce a trajectory
    traj, _, _ = dmp.plan()

    # Reproduction w/ visualization
    dmp.plot_traj(trajs_demo, np.expand_dims(traj, axis=0))

    
def problem_1d():
    """
    Train DMPs with multiple min-jerk trajectories and reproduce 
    the demo with different goals.
    """
    n_samples = 10
    dims      = 2
    bfs       = 20
    tau       = 1.
    
    # Data generation
    trajs_demo = min_jerk_trajs(dims=dims, n_samples=n_samples, add_noise=True)

    # Learn via DMP original/improved
    dmp = DMPs_discrete(dims=dims, bfs=bfs, tau=tau)
    dmp.learn(trajs_demo)

    # ReProduce a trajectory
    y0    = None
    trajs = []

    #------------------------------------------------------------
    # Place your code here
    for i in range(10):
        goal = np.array([1.0*((i-5)/2.0), 1.0*((i-5)/2.0)])
        traj, _, _ = dmp.plan(y0=y0, goal=goal)
        trajs.append(traj)
    #------------------------------------------------------------

    # Reproduction w/ visualization
    dmp.plot_traj(trajs_demo, np.array(trajs))
    


    
    



if __name__ == "__main__":
    import optparse
    p = optparse.OptionParser()
    p.add_option('--subproblem', '-s', action='store',
                 default='a', help='use this option to specify the subproblem')
    opt, args = p.parse_args()
    


    if opt.subproblem == 'a':
        # Train DMP with a 1-dimensional single demo and reproduce it (also plot basis functions)
        print("Problem 1A")
        problem_1a()
    elif opt.subproblem == 'b':
        # Adapt the DMP to other goals
        print("Problem 1B")
        problem_1b()
    elif opt.subproblem == 'c':
        # Train DMP with multiple demos and reproduce it 
        print("Problem 1C")
        problem_1c()
    elif opt.subproblem == 'd':
        # Adapt the DMP to other goals    
        print("Problem 1D")
        problem_1d()
    else:
        print("Error! Please specify the sub problem!")




    
    
