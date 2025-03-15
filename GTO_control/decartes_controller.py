import numpy as np

from synchronous_controller import SynchronousController
from arm_kinematics import ArmKinematics

from utils import (
    log, 
    construct_arm_message,
    SE3_from_xyz_rpy,
    tf_to_rpy
)


class DecartesController(SynchronousController):
    
    def __init__(self, is_in_local=False, network_interface=None):

        log(self, 'Initializing...')

        # initialize parent control
        SynchronousController.__init__(
            self, 
            network_interface=network_interface, 
            is_in_local=is_in_local
            )
        
        # initialize arm
        log(self, 'Initializin Arm Kinematix')
        self.arm = ArmKinematics()

    def go_to(self, l_pos=None, l_rpy=None, r_pos=None, r_rpy=None):
        # convert to SE3
        l_pose = SE3_from_xyz_rpy(l_pos, l_rpy)
        r_pose = SE3_from_xyz_rpy(r_pos, r_rpy)

        self._go_to_homo(l_pose.homogeneous, r_pose.homogeneous)

    def _go_to_homo(self, l_tf_homo, r_tf_homo):

        # get initial guess
        init_guess = np.asarray(self._GetJointStates())

        # solve inverse kinematics
        target_q = self.arm._solve_ik(
            l_wrist_target=l_tf_homo, 
            r_wrist_target=r_tf_homo, 
            current_lr_arm_motor_q=init_guess
            )
        
        # create message
        msg = construct_arm_message(target_q)

        # execute command
        self.ExecuteCommand(msg)

        
    def get_ee_xyzrpy(self):
        l_pose, r_pose = self.get_ee_poses()

        l_xyz = l_pose.translation
        r_xyz = r_pose.translation

        l_rpy = tf_to_rpy(l_pose)
        r_rpy = tf_to_rpy(r_pose)

        return (l_xyz, l_rpy), (r_xyz, r_rpy)

    def get_ee_poses(self):
        states = np.asarray(self._GetJointStates())
        l_pose, r_pose = self.arm._get_forward_kinematics(states)
        return l_pose, r_pose
