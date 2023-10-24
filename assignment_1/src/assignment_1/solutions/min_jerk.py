#!/usr/bin/env python
import numpy as np
import sys, copy

def min_jerk(start, goal, dur, freq=100):
    """ 
    A function to generate a minimum-jerk trajectory.

    Parameters
    ----------
    start : array
        a vector of start position (e.g., np.array([0,0,0]))
    goal  : array
        a vector of goal position (e.g., np.array([1,2,1]))
    dur   : float
        time duration of the resulted trajectory
    freq  : float
        time frequency (Hz)

    Returns
    -------
    time : array
           a list of progress from 0 to 1
    X    : array
           a position trajectory from a start to a goal
    Xd   : array
           a velocity trajectory
    Xdd  : array
           an acceleration trajectory
    Xddd : array
           a jerk trajectory
    """
    if type(start) in [list, tuple]: start = np.array(start)
    if type(goal) in [list, tuple]: goal = np.array(goal)
        
    D = len(start) # dimension
    P = int(freq * dur) # the number of points

    time = []
    X  = []
    Xd = []
    Xdd  = []
    Xddd = []
    for i in range(P):
        # ------------------------------------------------------
        # Place your code here
        t = float(i)/float(P-1) 

        time.append(t)
        X.append( start + (goal-start)*(10.0*(t)**3 - 15.*(t)**4 + 6*(t)**5) )
        Xd.append( (goal-start)/float(P)*(30.0*(t)**2 - 60.*(t)**3 + 30*(t)**4) )
        Xdd.append( (goal-start)/float(P)*(60.0*t - 180.*(t)**2 + 120*(t)**3) )
        Xddd.append( (goal-start)/float(P)*(60.0 - 360.*t + 360*(t)**2) )
        # ------------------------------------------------------

    return time, np.array(X), np.array(Xd), np.array(Xdd), np.array(Xddd)


if __name__ == '__main__':

    # start and goal
    start = np.array([-4.0, 10.0, 4.0])
    goal  = np.array([4.0, 4.0, 4.0])
    N,D   = 2, len(start)

    freq     = 100 #Hz 
    duration = 5.
    
    _, trj, trj_vel, trj_acc, trj_jerk = min_jerk(start, goal, duration, freq)

    # ------------------------------------------------------
    # Place your code here to visualize the trajectories
    length = len(trj)

    import matplotlib.pyplot as plt
    fig = plt.figure(1)
    
    for i in range(D):
        ax = fig.add_subplot(D, 4, i*4+1)
        plt.plot(range(length), trj[:,i], 'r-', markersize=12)

        ax = fig.add_subplot(D, 4, i*4+2)
        plt.plot(range(length), trj_vel[:,i], 'r-', markersize=12)

        ax = fig.add_subplot(D, 4, i*4+3)
        plt.plot(range(length), trj_acc[:,i], 'r-', markersize=12)

        ax = fig.add_subplot(D, 4, i*4+4)
        plt.plot(range(length), trj_jerk[:,i], 'r-', markersize=12)
        
    plt.show()
    # ------------------------------------------------------
    
