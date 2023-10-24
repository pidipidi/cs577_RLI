#!/usr/bin/env python3
import argparse
import time
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
import numpy as np
import baxter_interface
from baxter_interface import CHECK_VERSION

JOINT_NAMES = {'left': ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2'],
               'right': ['right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']}
positions = None
    
def state_callback(msg):
    ''' joint actual position callback '''
    global positions
    positions = msg.actual.positions

def get_joint_names(limb):
    ''' Get the joint names of the specified limb '''
    return JOINT_NAMES[limb]

def get_current_angles(limb):
    ''' Get the current joint angles '''

    # joint actual position
    global positions
    positions = None
    
    rospy.Subscriber('robot/'+limb+'_traj_action_controller/state',
                         JointTrajectoryControllerState,
                         state_callback)

    # 
    while not rospy.is_shutdown():
        if positions is not None: break
        rospy.sleep(1.0)

    return positions


def move(limb, points):
    ''' Move the joints of the limb following the trajectory points'''
    assert len(points)>=1, "points should be a list of waypoints"
            
    try:
        client = actionlib.SimpleActionClient('/robot/'+limb+'_traj_action_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo( "Waiting for action server..." )
        client.wait_for_server()
        rospy.loginfo( "Connected to action server" )
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
    rospy.loginfo("Initialized server")
    rospy.sleep(2)
    
    g = FollowJointTrajectoryGoal()
    g.goal_time_tolerance = rospy.Duration.from_sec(0.05)
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES[limb]
    g.trajectory.points = points
    g.trajectory.header.stamp = rospy.Time.now()

    client.send_goal(g)
    try:
        print(client.wait_for_result())
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

    rospy.sleep(4)
    rospy.loginfo("Moved")


def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='send joint trajectory to which limb'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    limb = args.limb

    print("Initializing node... ")
    rospy.init_node("joint_trajectory_client_%s" % (limb,))
    print("Getting robot state... ")
    #rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    #rs.enable()
    print("Running. Ctrl-c to quit")
    positions = {
        'left':  [[0.41, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39],[-0.41, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39]],
        'right':  [[0.11, -0.62,  1.15, 1.32, -0.80, 1.27, -2.39]],
    }

    points = []    
    t = 0
    for ang in positions[limb]:
        t += 10
        point = JointTrajectoryPoint()
        point.positions = ang
        point.velocities = [0]*7
        point.time_from_start = rospy.Duration.from_sec(t)                
        points.append(point)
        
    move(limb, points)

    print("Exiting - Joint Trajectory Action Test Complete")

if __name__ == "__main__":
    main()
