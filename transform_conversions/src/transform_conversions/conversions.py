#!/usr/bin/env python

import rospy, tf, numpy as np
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
import transform_conversions.transformations as transf


# Converts a 16x1 list to a 4x4 numpy matrix
def homogeneous_vector_to_matrix(vector):
    matrix = np.resize(vector, (4,4))
    if matrix[3,0] != 0.0 or matrix[3,1] != 0.0 or matrix[3,2] != 0.0:
        matrix = np.transpose(matrix)
    return matrix

# Converts a geometry_msgs/Pose into a 4x4 numpy matrix
def pose_msg_to_matrix(pose):
    assert isinstance(pose, Pose), 'Input is not of type geometry_msgs/Pose'
    matrix = transf.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    matrix[0:3,3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


def xyzrpy_to_matrix(data):
    assert len(data) == 6
    matrix = transf.euler_matrix(data[3], data[4], data[5])
    matrix[0:3,3] = [data[0], data[1], data[2]]
    return matrix

# Converts a geometry_msgs/PoseStamped into a 4x4 numpy matrix
def pose_stamped_msg_to_matrix(pose_stamped):
    assert isinstance(pose, PoseStamped), 'Input is not of type geometry_msgs/PoseStamped'
    return pose_msg_to_matrix(pose_stamped.pose)


# Converts a geometry_msgs/TransformStamped into a geometry_msgs/Pose
def transform_stamped_msg_to_pose_msg(transform_stamped_msg):
    assert isinstance(pose, TransformStamped), 'Input is not of type geometry_msgs/TransformStamped'
    position = transform_stamped_msg.transform.translation
    quaternion = transform_stamped_msg.transform.rotation
    pose_msg = Pose()
    pose_msg.position.x = position.x
    pose_msg.position.y = position.y
    pose_msg.position.z = position.z
    pose_msg.orientation.x = quaternion.x
    pose_msg.orientation.y = quaternion.y
    pose_msg.orientation.z = quaternion.z
    pose_msg.orientation.w = quaternion.w
    return pose_msg


# Converts a geometry_msgs/TransformStamped into a geometry_msgs/PoseStamped
def transform_stamped_msg_to_pose_stamped_msg(transform_stamped_msg):
    assert isinstance(pose, TransformStamped), 'Input is not of type geometry_msgs/TransformStamped'
    pose_stamped_msg = PoseStamped()
    pose_stamped_msg.header = transform_stamped_msg.header
    pose_stamped_msg.pose = transform_stamped_msg_to_pose_msg(transform_stamped_msg)
    return pose_stamped_msg


# Converts a geometry_msgs/TransformStamped into a 4x4 numpy matrix
def transform_stamped_msg_to_matrix(transform_stamped_msg):
    assert isinstance(transform_stamped_msg, TransformStamped), 'Input is not of type geometry_msgs/TransformStamped'
    position = transform_stamped_msg.transform.translation
    quaternion = transform_stamped_msg.transform.rotation

    return np.dot(transf.translation_matrix([position.x, position.y, position.z]), \
				transf.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w]))


# Converts a geometry_msgs/Pose into a list [x,y,z,q1,q2,q3,q4]
def pose_msg_to_pose_vector(pose):
    assert isinstance(pose, Pose), 'Input is not of type geometry_msgs/Pose'
    return [pose.position.x, pose.position.y, pose.position.z, \
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]


# Converts a pose_vector (list of length 7: x,y,z,q1,q2,q3,q4) into a geometry_msgs/Pose
def pose_vector_to_pose_msg(pose_vector):
    assert len(pose_vector) == 7, 'Invalid pose_vector. Should be length 7 (x,y,z,q1,q2,q3,q4).'
    pose_msg = Pose()
    pose_msg.position.x = pose_vector[0]
    pose_msg.position.y = pose_vector[1]
    pose_msg.position.z = pose_vector[2]
    pose_msg.orientation.x = pose_vector[3]
    pose_msg.orientation.y = pose_vector[4]
    pose_msg.orientation.z = pose_vector[5]
    pose_msg.orientation.w = pose_vector[6]
    return pose_msg


def pose_vector_to_matrix(pose_vector):
    assert len(pose_vector) == 7, 'Invalid pose_vector. Should be length 7 (x,y,z,q1,q2,q3,q4).'
    return pose_msg_to_matrix(pose_vector_to_pose_msg(pose_vector))

# Converts a 4x4 numpy matrix into a geometry_msgs/Pose
def matrix_to_pose_msg(matrix):
    position_vector = list(transf.translation_from_matrix(matrix))
    quaternion_vector = list(transf.quaternion_from_matrix(matrix))
    quaternion_vector = [quaternion_vector[1], quaternion_vector[2], quaternion_vector[3], quaternion_vector[0]]
    return pose_vector_to_pose_msg(position_vector+quaternion_vector)


# Converts a 4x4 numpy matrix into a list [x,y,z,q1,q2,q3,q4]
def matrix_to_pose_vector(matrix):
    return pose_msg_to_pose_vector(matrix_to_pose_msg(matrix))

# Converts a 4x4 numpy matrix and associated frame_id into a geometry_msgs/PoseStamped
def matrix_to_pose_stamped_msg(matrix, frame_id):
    pose_stamped_msg = PoseStamped()
    pose_stamped_msg.header.frame_id = frame_id
    pose_stamped_msg.header.stamp = rospy.Time.now()
    pose_stamped_msg.pose = matrix_to_pose_msg(matrix)
    return pose_stamped_msg


# Broadcasts a ROS TF describing the transformation matrix from source_frame to target_frame
def broadcast_matrix_as_tf(matrix, source_frame, target_frame):
    position = tuple(transf.translation_from_matrix(matrix))
    quaternion = tuple(transf.quaternion_from_matrix(matrix))
    quaternion = (quaternion[1], quaternion[2], quaternion[3], quaternion[0]) # (x, y, z, w)
    tf.TransformBroadcaster().sendTransform(position, quaternion, rospy.Time.now(), target_frame, source_frame)
