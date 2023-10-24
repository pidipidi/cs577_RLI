#!/usr/bin/env python3
import numpy as np
import min_jerk as mj
from dmp import *

def gen_circular_traj(freq=100, duration=1.):

    n_steps     = int(duration*freq)
    n_ext_steps = int(duration*0.1*freq)
    max_ang = 2.0*np.pi - np.pi/10.0
    r       = 0.5
    cen     = [0, r]
    
    _, trj, _, _, _ = mj.min_jerk([0], [1], duration, freq)
    
    y = np.zeros((1, 2, n_steps+n_ext_steps))
    for i in range(n_steps):
        y[0,:,i] = np.array([ cen[0]+r*np.sin(-np.pi+trj[i,0]*max_ang) ,
                                  cen[1]+r*np.cos(-np.pi+trj[i,0]*max_ang) ])

    for i in range(n_steps-1,n_steps+n_ext_steps):
        y[0,:,i] = y[0,:,n_steps-1]                
    return y


def plot_2d(traj, trajs_demo):
    # Reproduction w/ visualization
    fig = plt.figure()
    plt.title('Trajectory (X) - Demo (Td) and generated (Tg)')
    plt.plot(trajs_demo[0,0], trajs_demo[0,1], 'r--', label='Td')
    plt.plot(traj[0], traj[1], 'g-', label='Tg')
    plt.legend()
    plt.show()        
    

def problem_2a1():
    """
    Train an original DMP with a circular traj and reproduce it with different goals.
    """    
    n_samples = 1
    dims      = 2
    bfs       = 30
    tau       = 1.

    freq      = 100
    duration  = 1.
    dt        = 1./freq
    
    # Data generation
    trajs_demo = gen_circular_traj(freq=freq, duration=duration)
    print ("start:", trajs_demo[0, :, 0])
    print ("goal:", trajs_demo[0,:, -1])

    # Learn via DMP original/improved
    dmp = DMPs_discrete(dims=dims, bfs=bfs, tau=tau, dt=dt,
                            enable_improved=False)
    traj, _, _ = dmp.learn(trajs_demo)

    # ReProduce a trajectory
    y0   = None
    goal = None
    
    traj, _, _ = dmp.plan(y0=y0, goal=goal)
    plot_2d(traj, trajs_demo)


def problem_2a2():
    """
    Train an improved DMP with a circular traj and reproduce it with different goals.
    """    
    n_samples = 1
    dims      = 2
    bfs       = 30
    tau       = 1.

    freq      = 100
    duration  = 1.
    dt        = 1./freq
    
    # Data generation
    trajs_demo = gen_circular_traj(freq=freq, duration=duration)
    print ("start:", trajs_demo[0, :, 0])
    print ("goal:", trajs_demo[0,:, -1])

    # Learn via DMP original/improved
    dmp = DMPs_discrete(dims=dims, bfs=bfs, tau=tau, dt=dt,
                            enable_improved=True)
    traj, _, _ = dmp.learn(trajs_demo)

    # ReProduce a trajectory
    y0   = None
    goal = None
    
    traj, _, _ = dmp.plan(y0=y0, goal=goal)
    plot_2d(traj, trajs_demo)

    
def problem_2b():
    """
    Train an original DMP with a circular traj and reproduce it with different goals.
    """    
    n_samples = 1
    dims      = 2
    bfs       = 30
    tau       = 1.

    freq      = 100
    duration  = 1.
    dt        = 1./freq
    
    # Data generation
    trajs_demo = gen_circular_traj(freq=freq, duration=duration)
    print ("start:", trajs_demo[0, :, 0])
    print ("goal:", trajs_demo[0,:, -1])

    # Learn via DMP original/improved
    dmp = DMPs_discrete(dims=dims, bfs=bfs, tau=tau, dt=dt,
                            enable_improved=False)
    traj, _, _ = dmp.learn(trajs_demo)

    #------------------------------------------------------------
    # Place your code here
    # ReProduce a trajectory
    y0   = None #np.array([-0.5,])
    goal = np.array([0.15,-0.12])
    #------------------------------------------------------------
    
    traj, _, _ = dmp.plan(y0=y0, goal=goal)
    plot_2d(traj, trajs_demo)
                
    
       
if __name__ == "__main__":
    import optparse
    p = optparse.OptionParser()
    p.add_option('--subproblem', '-s', action='store',
                 default='a', help='use this option to specify the subproblem')
    opt, args = p.parse_args()

    if opt.subproblem == 'a1':
        # Train an original DMP with a 2-dimensional single demo and reproduce it.
        print("Problem 2A-1")
        problem_2a1()
    elif opt.subproblem == 'a2':    
        # Train an improved DMP with a 2-dimensional single demo and reproduce it.
        print("Problem 2A-2")
        problem_2a2()
    elif opt.subproblem == 'b':        
        # Vary the goals and compare the difference between the previous two DMPs
        print("Problem 2B")
        problem_2b()
    else:
        print("Error! Please specify the sub problem!")


    
