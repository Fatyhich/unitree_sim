import numpy as np
from time import sleep
import pinocchio as pin
from utils.arm_definitions import G1JointArmIndex, G1JointIndex

def log(object, msg):
    print(f'[{type(object).__name__}] {msg}')

def construct_arm_message(joint_states, joint_velocities=None, torq_ff=None):
    
    msg = {}

    # if dq is None, use zeros
    if joint_velocities is None:
        joint_velocities = np.zeros_like(joint_states)

    # if torqs are None, use zeros
    if torq_ff is None:
        torq_ff = np.zeros_like(joint_states)

    # fill message
    for idx, joint in enumerate(G1JointArmIndex):
        msg[joint] = (joint_states[idx], joint_velocities[idx], torq_ff[idx])

    return msg

def SE3_from_xyz_rpy(xyz, rpy):
    rot_matr = pin.rpy.rpyToMatrix(rpy)
    return pin.SE3(rot_matr, xyz)

def SE3_to_xyzrpy(se3_object):
    xyz = se3_object.translation
    rpy = tf_to_rpy(se3_object)
    return xyz, rpy

def tf_to_rpy(tf):
    rot_matrix = tf.rotation
    rpy = pin.rpy.matrixToRpy(rot_matrix)
    return rpy

def get_rpy_component(tf, component_idx):
    return tf_to_rpy(tf)[component_idx]

def set_rpy_component(tf, component_idx, new_value):
    rpy = tf_to_rpy(tf)
    rpy[component_idx] = new_value
    tf.rotation = pin.rpy.rpyToMatrix(rpy)
    return tf
