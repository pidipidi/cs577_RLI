#!/usr/bin/env python

import math
import numpy as np
from geometry_msgs.msg import Point, Quaternion, Pose
import PyKDL


def array2KDLframe(pose_array):

    if len(pose_array)==6:
        p = PyKDL.Vector(pose_array[0], pose_array[1], pose_array[2])
        M = PyKDL.Rotation.RPY(pose_array[3], pose_array[4], pose_array[5])
    elif len(pose_array)==7:
        p = PyKDL.Vector(pose_array[0], pose_array[1], pose_array[2])
        M = PyKDL.Rotation.Quaternion(pose_array[3], pose_array[4], \
                                      pose_array[5], pose_array[6])
    else:
        print ("[ERROR] Wrong length of pose_array input")

    return PyKDL.Frame(M,p)

def mat2KDLframe(mat):
    p = PyKDL.Vector(mat[0,3],mat[1,3],mat[2,3])
    M = PyKDL.Rotation(mat[0,0],mat[0,1],mat[0,2], mat[1,0],mat[1,1],mat[1,2], \
                       mat[2,0],mat[2,1],mat[2,2] )
    return PyKDL.Frame(M,p)

def pose2KDLframe(pose):
    p = PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z)
    if pose.orientation.x == 0 and pose.orientation.y == 0 and pose.orientation.z == 0 and\
      pose.orientation.w == 0:
        M = PyKDL.Rotation.Identity()
    else:
        M = PyKDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, \
                                      pose.orientation.z, pose.orientation.w)
        
    return PyKDL.Frame(M, p)

def pose2array(pose):
    return np.array([pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, \
            pose.orientation.z, pose.orientation.w] )

def pose2list(pose):
    return (pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, \
            pose.orientation.z, pose.orientation.w )

def KDLframe2Pose(frame):

    ps = Pose()

    ps.position.x = frame.p[0]
    ps.position.y = frame.p[1]
    ps.position.z = frame.p[2]

    ps.orientation.x = frame.M.GetQuaternion()[0]
    ps.orientation.y = frame.M.GetQuaternion()[1]
    ps.orientation.z = frame.M.GetQuaternion()[2]
    ps.orientation.w = frame.M.GetQuaternion()[3]

    return ps

def KDLframe2Array(frame):
    return np.array([frame.p[0], frame.p[1], frame.p[2], \
                     frame.M.GetQuaternion()[0], frame.M.GetQuaternion()[1], \
                     frame.M.GetQuaternion()[2], frame.M.GetQuaternion()[3] ])

def KDLframe2List(frame):
    return [frame.p[0], frame.p[1], frame.p[2], \
            frame.M.GetQuaternion()[0], frame.M.GetQuaternion()[1], \
            frame.M.GetQuaternion()[2], frame.M.GetQuaternion()[3] ]

def list2KDLframe(l):
    return array2KDLframe(np.array(l))

def list2Pose(l):

    ps = Pose()

    if len(l)==0: return ps
    
    ps.position.x = l[0]
    ps.position.y = l[1]
    ps.position.z = l[2]

    if len(l)==6:
        q = PyKDL.Rotation.RPY(l[3], l[4], l[5]).GetQuaternion()
        ps.orientation=Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])
    else:
        ps.orientation=Quaternion(x=l[3],y=l[4],z=l[5],w=l[6])

    return ps
    

def posedict2List(pose_dict):

    return [pose_dict['position'].x,
            pose_dict['position'].y,
            pose_dict['position'].z,
            pose_dict['orientation'].x,
            pose_dict['orientation'].y,
            pose_dict['orientation'].z,
            pose_dict['orientation'].w]
    
def posedict2Pose(pose_dict):
    return list2Pose(posedict2List(pose_dict))

def list_quat2list_rpy(s):
    m = PyKDL.Rotation.Quaternion(s[3], s[4], s[5], s[6])
    rpy = m.GetRPY()
    return [s[0], s[1], s[2], rpy[0], rpy[1], rpy[2]]
def list_rpy2list_quat(s):
    m = PyKDL.Rotation.RPY(s[3], s[4], s[5])
    quat = m.GetQuaternion()
    return [s[0], s[1], s[2], quat[0], quat[1], quat[2], quat[3]]


def KDLframe2Matrix(frame):
    Rx = frame.M.UnitX(); Ry = frame.M.UnitY(); Rz = frame.M.UnitZ()
    return np.array([[Rx[0], Ry[0], Rz[0], frame.p[0]],
                         [Rx[1], Ry[1], Rz[1], frame.p[1]],
                         [Rx[2], Ry[2], Rz[2], frame.p[2]],
                         [0,0,0,1]]) 

# -----------------------------------------------------------------------------------------
def dist_positions(position1, position2):

    return np.linalg.norm(np.array([position1.x,
                                    position1.y,
                                    position1.z])
                        -np.array([position2.x,
                                   position2.y,
                                   position2.z]))

def dist_quaternions(quat1, quat2):
    """angle between two quaternions (as lists)"""
    if type(quat1) == Quaternion:
        quat1 = [quat1.x, quat1.y, quat1.z, quat1.w]
    if type(quat2) == Quaternion:
        quat2 = [quat2.x, quat2.y, quat2.z, quat2.w]
            
    dot = math.fabs(sum([x*y for (x,y) in zip(quat1, quat2)]))
    if dot > 1.0: dot = 1.0
    angle = 2*math.acos( dot )
    return angle 
    
## def angle_vectors(v1, v2):
##     def dotproduct(v1, v2):
##         return sum((a*b) for a, b in zip(v1, v2))

##     def length(v):
##         return math.sqrt(dotproduct(v, v))

##     return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))

def angle_vectors(v1, v2):
    """
    vector is (x,y)
    """
    if type(v1) is not np.ndarray: v1 = np.array(v1)
    if type(v2) is not np.ndarray: v2 = np.array(v2)
        
    v1u = v1/np.linalg.norm(v1)
    v2u = v2/np.linalg.norm(v2)
    angle = np.arctan2(v1u[0]*v2u[1]-v1u[1]*v2u[0],v1u[0]*v2u[0]+v1u[1]*v2u[1])
    return angle
