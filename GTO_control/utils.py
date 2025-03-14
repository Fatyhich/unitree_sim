import numpy as np
import pinocchio as pin

from arm_definitions import G1JointArmIndex

def log(object, msg):
    print(f'[{type(object).__name__}] {msg}')

def construct_arm_message(joint_states, joint_velocities=None):
    
    msg = {}

    # if dq is None, use zeros
    if joint_velocities is None:
        joint_velocities = np.zeros_like(joint_states)

    # fill message
    for idx, joint in enumerate(G1JointArmIndex):
        msg[joint] = (joint_states[idx], joint_velocities[idx])

    return msg

def SE3_from_xyz_rpy(xyz, rpy):
    rot_matr = pin.rpy.rpyToMatrix(rpy)
    return pin.SE3(rot_matr, xyz)


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


        
