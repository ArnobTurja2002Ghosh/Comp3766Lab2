#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from core import IKinBody, RpToTrans
import pandas as pd

def compute_ik(position, orientation):
    """
    Function that calulates the PUMA analytical IK function.
    Should take a 3x1 position vector and a 3x3 rotation matrix,
    and return a list of joint positions.
    """
    
    T = RpToTrans(orientation, position)
    #print("Received Transformation", T)
    Blist = np.array([[0, -1, 0, 0.15, 0,   -0.864],
                [0, 0, -1, 0, 0.864,   0],
                [0, 0, -1, 0, 0.432, 0],
                [0, -1, 0, 0, 0, 0],
                [-1, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0]]).T
    M = np.array([  [ 0,    0, 1, -0.15],
                    [-1,    0, 0, 0.864],
                    [ 0,   -1, 0,     0],
                    [ 0,    0, 0,     1]])
    # T = np.array([  [0,    0, 1,   -0.15],
    #                 [-1,   0, 0,     0.864],
    #                 [0,   -1, 0,        0],
    #                 [0,    0, 0,        1]])
    thetalist0 = np.array([0, 0, 0, 0, 0, 0])
    eomg = 0.001
    ev = 0.001
    return IKinBody(Blist, M, T, thetalist0, eomg, ev)[0]

def pose_callback(msg):
    """
    Callback function to handle incoming end-effector pose messages.
    You probably do not have to change this
    """
    # Extract position (3x1)
    position = np.array([msg.position.x, msg.position.y, msg.position.z])

    # Extract orientation (3x3 rotation matrix from quaternion)
    q = msg.orientation
    orientation = np.array([
        [1 - 2 * (q.y**2 + q.z**2), 2 * (q.x*q.y - q.z*q.w), 2 * (q.x*q.z + q.y*q.w)],
        [2 * (q.x*q.y + q.z*q.w), 1 - 2 * (q.x**2 + q.z**2), 2 * (q.y*q.z - q.x*q.w)],
        [2 * (q.x*q.z - q.y*q.w), 2 * (q.y*q.z + q.x*q.w), 1 - 2 * (q.x**2 + q.y**2)]
    ])

    # Compute inverse kinematics
    joint_positions = compute_ik(position, orientation)

    # Publish joint states
    joint_msg = JointState()
    joint_msg.header.stamp = rospy.Time.now()
    joint_msg.name = [f"joint{i+1}" for i in range(len(joint_positions))]
    joint_msg.position = joint_positions
    joint_pub.publish(joint_msg)

if __name__ == "__main__":
    rospy.init_node("ik_solver_node", anonymous=True)

    # Publisher: sends joint positions
    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    
    print("\nWaiting for /goal_pose")
    
    # Subscriber: listens to end-effector pose
    rospy.Subscriber("/goal_pose", Pose, pose_callback)

    rospy.spin()
