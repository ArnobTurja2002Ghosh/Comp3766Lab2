#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

def compute_ik(position, orientation):
    """
    Function that calulates the PUMA analytical IK function.
    Should take a 3x1 position vector and a 3x3 rotation matrix,
    and return a list of joint positions.
    """
    
    print("\nReceived Position:")
    print(position)

    print("\nReceived Orientation (3x3 Rotation Matrix):")
    print(orientation)

    # PUMA Robot Parameters (meters)
    d1, a2, a3 = 0.150, 0.432, 0.432  # Given parameters
    
    # Replace with the actual analytical IK computation
    # Insert you code here
    
    # Comment this line when you have your solution
    theta1, theta2, theta3, theta4, theta5, theta6 = [0.0] * 6
    end_effector_pos=position
    end_effector_ori=orientation
    joint_angles = np.zeros(6)  # Placeholder for the actual joint angles, comment this line
    joint_angles[0]= np.arctan2(-end_effector_pos[0],end_effector_pos[1])-np.arctan2(d1, np.sqrt(end_effector_pos[0]**2+end_effector_pos[1]**2-d1**2))
    #print((end_effector_pos[0]**2+end_effector_pos[1]**2+end_effector_pos[2]**2-d1**2-a2**2-a3**2)/(2*a2*a3))
    joint_angles[2]= np.arccos((end_effector_pos[0]**2+end_effector_pos[1]**2+end_effector_pos[2]**2-d1**2-a2**2-a3**2)/(2*a2*a3))
    joint_angles[1]= -np.arctan2(end_effector_pos[2],np.sqrt(end_effector_pos[0]**2+end_effector_pos[1]**2-d1**2))-np.arctan2(a3*np.sin(joint_angles[2]),a2+a3*np.cos(joint_angles[2]))
    #joint_angles[3:6]=np.arccos(np.dot(end_effector_ori[:,0],np.array([0,0,1]))),np.arccos(np.dot(end_effector_ori[:,1],np.array([0,0,1]))),np.arccos(np.dot(end_effector_ori[:,2],np.array([0,0,1])))
    
    R=np.array([
        [1,0,0],
        [0, np.cos(joint_angles[2]), -np.sin(joint_angles[2])],
        [0, np.sin(joint_angles[2]), np.cos(joint_angles[2])]
    ]) @ np.array([
        [1,0,0],
        [0, np.cos(joint_angles[1]), -np.sin(joint_angles[1])],
        [0, np.sin(joint_angles[1]), np.cos(joint_angles[1])]
    ]) @ np.array([
        [np.cos(joint_angles[0]), np.sin(joint_angles[0]), 0],
        [-np.sin(joint_angles[0]), np.cos(joint_angles[0]), 0],
        [0,0,1] 
    ])@end_effector_ori@np.array([
        [0, 0, 1],
        [-1, 0, 0],
        [0, -1, 0]
    ]).transpose()
    
    #print(R)
    #print(np.cos(np.arctan2(R[1][0], R[0][0]))*np.cos(np.arctan2(-R[2][0], np.sqrt(R[0][0]**2 + R[1][0]**2))))
    
    joint_angles[3]= np.arctan2(R[1][0], R[0][0])
    joint_angles[4]= np.arctan2(-R[2][0], np.sqrt(R[0][0]**2 + R[1][0]**2))
    joint_angles[5]=np.arctan2(R[2][1], R[2][2])
    print(joint_angles)
    return joint_angles

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
