import numpy as np

from GTO_control.controllers.synchronous_controller import SynchronousController
from GTO_control.kinematics.single_arm_kinematics import SingleArmKinematics

from GTO_control.utils.utils import (
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
        log(self, 'Initializing Arm Kinematix')
        self.l_arm = SingleArmKinematics(is_left=True)
        self.r_arm = SingleArmKinematics(is_left=False)
        print('Decartes Controller Initialized')

    def get_initial_guess(self):
        return np.asarray(self._GetLeftJoints()), np.asarray(self._GetRightJoints())

    def go_to(self, l_pos, l_rpy, r_pos, r_rpy):
        # get initial guess
        l_init_guess, r_init_guess = self.get_initial_guess()

        # calculate ik
        l_target_q = self.l_arm.inverse_kinematics(
            xyz=l_pos,
            rpy=l_rpy,
            current_motor_q=l_init_guess
        )

        r_target_q = self.r_arm.inverse_kinematics(
            xyz=r_pos,
            rpy=r_rpy,
            current_motor_q=r_init_guess
        )

        msg = construct_arm_message(np.concatenate((l_target_q, r_target_q)))

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
        l_pose = self.l_arm.get_ee_pose(states[:7])
        r_pose = self.r_arm.get_ee_pose(states[7:])
        return l_pose, r_pose
