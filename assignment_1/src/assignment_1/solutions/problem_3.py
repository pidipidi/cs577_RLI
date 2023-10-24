#!/usr/bin/env python3
import os
import numpy as np
import json
import matplotlib.pylab as plt
import rospy

import misc
from dmp import *
import joint_trajectory_client as jtc
from trajectory_msgs.msg import *
## import baxter_interface
## from baxter_interface import CHECK_VERSION

from pykdl_utils.kdl_kinematics import create_kdl_kin
from trac_ik_python.trac_ik import IK  
from hrl_geom.pose_converter import PoseConv

def extract_data():
    """
    Extract a pouring motion demo from the MIME dataset. (you can download other demo. if you want.)
    """    
    cur_dir = os.getcwd().split('/')[-1]
    assert cur_dir=='assignment_1', "Run the program on the assignment_1 folder. Current directory is {}".format(cur_dir)

    if os.path.isdir('dataset') is False:
        os.mkdir('dataset')
    os.chdir( 'dataset' )
                
    url = 'https://www.dropbox.com/sh/wmyek0jhrpm0hmh/AADAO2L1qN5BwOBthyMG82ima/4315Aug02?dl=0.zip'
    if os.path.isfile(url.split('/')[-1]) is False:
        os.system('wget '+url)

    if os.path.isfile('joint_angles.txt') is False:
        os.system('unzip 4315Aug02?dl=0.zip')

    print (os.getcwd())
    data = []
    for line in open('joint_angles.txt', 'r'):
        data.append( json.loads(line))
    
    # Get left/right arm trajectories
    joint_names = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']

    l_arm_traj = []
    r_arm_traj = []
    for d in data:
        v = []
        for name in joint_names:
            v.append(d['left_'+name])
        l_arm_traj.append(v)

        v = []
        for name in joint_names:
            v.append(d['right_'+name])
        r_arm_traj.append(v)
        
    os.chdir( '..' )
    return np.swapaxes(r_arm_traj, 0, 1)


def fk_request(fk_solver, joint_angle):
    '''
    Forward kinematics that returns a desired pose
        
    fk_solver object: forward kinematics from KDL.
    joint_angle list/array: a list of joint target joint angles
    '''
    homo_mat = fk_solver.forward(joint_angle)
    pos, quat = PoseConv.to_pos_quat(homo_mat)
    return pos, quat


def ik_request(ik_solver, poses, seed_angle):
    '''
    Inverse kinematics that returns a sequence of desired joint angles
        
    ik_solver object: inverse kinematics from TRAC_IK.
    poses Pose: a sequence of target poses
    seed_angle list/array: reference angles (for nullspace control)
    '''

    if type(poses) is not list:
        poses = [poses]

    joint_positions = []
    for i, ps in enumerate(poses):
        if i%10==0: print (i, len(poses))
        ret = ik_solver.get_ik(seed_angle,
                                   ps.position.x,    
                                   ps.position.y,    
                                   ps.position.z,
                                   ps.orientation.x,
                                   ps.orientation.y,
                                   ps.orientation.z,
                                   ps.orientation.w,
                                   bx=1e-4, by=1e-4, bz=1e-4,
                                   brx=1e-4, bry=1e-4, brz=1e-4)
        
        
        if ret is None:
            continue
        seed_angle = ret
        ##       rospy.logerr("INVALID POSE - No Valid Joint Solution Found in {}th pose.".format(i))
        ##       return False

        joint_positions.append(ret)

    return np.swapaxes(joint_positions, 0,1)



def plot_traj(trajs, trajs_demo=None):
    """
    """
    
    fig = plt.figure()

    for i, traj in enumerate(trajs):
        fig.add_subplot(len(trajs), 1, i+1)
        plt.plot(traj, label=str(i))

        if trajs_demo is not None:
            plt.plot(trajs_demo[0][i], 'r-', label=str(i))
    
    plt.show()


def problem_3a1(enable_plot=True):
    traj = extract_data()

    dims      = len(traj)
    bfs       = 30
    tau       = 1.
    freq      = 100
    duration  = 1.
    dt        = 1./freq

    traj_demo = np.expand_dims(traj, axis=0)

    # Learn via DMP original/improved
    dmp = DMPs_discrete(dims=dims, bfs=bfs, tau=tau, dt=dt,
                            enable_improved=True)
    traj, _, _ = dmp.learn(traj_demo)
    traj, _, _ = dmp.plan()

    if enable_plot: plot_traj(traj, traj_demo)
    return traj
    

def problem_3a2(limb='right'):

    dt = 0.01
    
    print("Initializing node... ")
    rospy.init_node("rsdk_joint_trajectory_client_%s" % (limb,))
    
    #r_traj_data = extract_data()
    r_traj_data = problem_3a1(enable_plot=False)
    # print (jtc.get_joint_names(limb))


    # Command Current Joint Positions first
    points = []
    current_angles = jtc.get_current_angles(limb)
    point = JointTrajectoryPoint()
    point.positions = current_angles
    point.velocities = [0]*7
    point.time_from_start = rospy.Duration.from_sec(0.0)
    points.append(point)

    point = JointTrajectoryPoint()
    point.positions = r_traj_data[:,0]
    point.velocities = [0]*7
    point.time_from_start = rospy.Duration.from_sec(4.0)
    points.append(point)
    
    print("Waiting at the initial posture")
    jtc.move(limb, points)
    
    t = 0.
    points = []    
    #------------------------------------------------------------
    # Place your code here
    for i in range(len(r_traj_data[0])):
        t += dt
        point = JointTrajectoryPoint()
        point.positions = r_traj_data[:,i]
        point.velocities = [0]*7
        point.time_from_start = rospy.Duration.from_sec(t)
        points.append(point)

    point = JointTrajectoryPoint()
    point.positions = r_traj_data[:,-1]
    point.velocities = [0]*7
    point.time_from_start = rospy.Duration.from_sec(t+4.0)
    points.append(point)
    #------------------------------------------------------------

    # Run the trajectory
    jtc.move(limb, points)
    print("Exiting - Joint Trajectory Action Complete")

        
def problem_3bc(limb='right', goal=None):
    """
    Check if you want better orientation-based DMP: 
    Ude et al. Orientation in cartesian space dynamic movement primitives. In IEEE International Conference on Robotics and Automation (ICRA), 2014.
    """
    dims      = 7
    bfs       = 30
    tau       = 1.
    freq      = 100
    duration  = 1.
    dt        = 1./freq
    sample_scale = 20
    r_traj_data  = extract_data()
        
    # FK, IK solvers for Baxter
    fk_solver = create_kdl_kin('base', 'right_gripper')
    ik_solver = IK('base', 'right_gripper', timeout=0.025, epsilon=1e-3, solve_type="Distance")

    #------------------------------------------------------------
    # Place your code here
    # make a list of x,y,z,qx,qy,qz,qw
    pose_list = []
    for i in range(len(r_traj_data[0])):
        pos, quat = fk_request(fk_solver, r_traj_data[:,i])
        pose_list.append( list(pos)+list(quat) )
    pose_list = np.swapaxes(pose_list,0,1)
    traj_demo = np.expand_dims(pose_list, axis=0)
    #------------------------------------------------------------

    # Learn via DMP original/improved
    dmp = DMPs_discrete(dims=dims, bfs=bfs, tau=tau, dt=dt,
                            enable_improved=True)
    traj, _, _ = dmp.learn( traj_demo ) #[:,:6,:] )

    # setting a goal
    traj, _, _ = dmp.plan(goal=goal)

    # normalize the quaternions
    traj[3:] /= np.sum( traj[3:]**2, axis=0)
    
    # conver the pos+quaternion trajectory to pose list
    pose_list = []
    for i in range(len(traj[0])):
        # reduce the number of samples (OPTION)
        if i%sample_scale==0:
            ps = misc.list2Pose(traj[:,i])
            pose_list.append( ps )
        
    print("Initializing node... ")
    rospy.init_node("rsdk_joint_trajectory_client_%s" % (limb,))

    # Command Current Joint Positions first
    points = []
    current_angles = jtc.get_current_angles(limb)
    point = JointTrajectoryPoint()
    point.positions = current_angles
    point.velocities = [0]*7
    point.time_from_start = rospy.Duration.from_sec(0.)
    points.append(point)

    point = JointTrajectoryPoint()
    point.positions = r_traj_data[:,0]
    point.velocities = [0]*7
    point.time_from_start = rospy.Duration.from_sec(4.0)
    points.append(point)    
    jtc.move(limb, points)
        
    # Inverse Kinematics
    current_angles = jtc.get_current_angles(limb)
    for i in range(10):
        joint_des_traj = ik_request(ik_solver, pose_list, current_angles)
        if joint_des_traj is not False: break

    if joint_des_traj is False:
        rospy.logerr("Maybe unreachable goal pose... ")
        sys.exit()

    t = 0.
    points = []    
    ## input("Press <Enter> to Continue...")
    #------------------------------------------------------------
    # Place your code here
    current_angles = jtc.get_current_angles(limb)
    
    # Smoothing the output joint trajectory
    new_jnt_pos_l = []
    filter_size   = 15
    for i in range(len(joint_des_traj)):
        new_jnt_pos = [current_angles[i]]*filter_size + list(joint_des_traj[i]) + [joint_des_traj[i][-1]]*filter_size*2
        jnt_pos = np.convolve(new_jnt_pos,np.ones((filter_size,))/float(filter_size), mode='valid')
        new_jnt_pos_l.append(jnt_pos)
    joint_des_traj = np.array(new_jnt_pos_l)


    for i in range(len(joint_des_traj[0])):
        t += dt*sample_scale        
        point            = JointTrajectoryPoint()
        point.positions  = joint_des_traj[:,i]
        point.velocities = [0]*7
        point.time_from_start = rospy.Duration.from_sec(t)
        points.append(point)

    point = JointTrajectoryPoint()
    point.positions = joint_des_traj[:,-1]
    point.velocities = [0]*7
    point.time_from_start = rospy.Duration.from_sec(t+4.0)
    points.append(point)
    #------------------------------------------------------------
    
    # Run the trajectory
    jtc.move(limb, points)
    print("Exiting - Joint Trajectory Action Test Complete")
        
    
if __name__ == "__main__":
    import optparse
    p = optparse.OptionParser()
    p.add_option('--subproblem', '-s', action='store',
                 default='a1', help='use this option to specify the subproblem')
    opt, args = p.parse_args()

    if opt.subproblem == 'a1':
        # Obtain data
        # Train a DMP with high-dimensional joint-space data and reproduce it
        print("Problem 3A-1")
        problem_3a1()
    
    elif opt.subproblem == 'a2':
        # Reproduce it with a Baxter robot
        print("Problem 3A-2")
        problem_3a2()

    elif opt.subproblem == 'b':
        # Train a DMP with converted Cartesian-space data and reproduce it
        # Reproduce it with a Baxter robot
        print("Problem 3B")
        problem_3bc()
    
    elif opt.subproblem == 'c':
        # Train a DMP with converted Cartesian-space data and reproduce it
        # Reproduce it with a Baxter robot
        print("Problem 3C")
        # Adapt the goal to multiple poses and reproduce it with the Baxter robot

        #------------------------------------------------------------
        # Place your code here. Following is an example
        goal = [ 0.5319781 , -0.37729777,  0.04209853,  0.62513343,  0.46658677, 0.46685764, -0.41659203]
        #------------------------------------------------------------
        
        problem_3bc(goal=goal)
