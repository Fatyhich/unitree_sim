import numpy as np

from synchronous_controller import SynchronousController
from single_arm_kinematics import SingleArmKinematics

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
        log(self, 'Initializing Arm Kinematix')
        self.l_arm = SingleArmKinematics(is_left=True)
        self.r_arm = SingleArmKinematics(is_left=False)
        print('Decartes Controller Initialized')

    def go_to(self, l_pos, l_rpy, r_pos, r_rpy):
        # convert to SE3
        l_pose = SE3_from_xyz_rpy(l_pos, l_rpy)
        r_pose = SE3_from_xyz_rpy(r_pos, r_rpy)

        self._go_to_homo(l_pose.homogeneous, r_pose.homogeneous)

    def _go_to_homo(self, l_tf_homo, r_tf_homo):

        # get initial guess
        l_init_guess = np.asarray(self._GetLeftJoints())
        r_init_guess = np.asarray(self._GetRightJoints())

        # solve inverse kinematics for left 
        l_target_q = self.l_arm._solve_ik(
            wrist_target=l_tf_homo, 
            current_arm_motor_q=l_init_guess
            )

        # solve inverse kinematics for right
        r_target_q = self.r_arm._solve_ik(
            wrist_target=r_tf_homo, 
            current_arm_motor_q=r_init_guess
            )

        # create message
        msg = construct_arm_message(np.concatenate((l_target_q, r_target_q)))

        # execute command
        self.ExecuteCommand(msg)

        
    def get_ee_xyzrpy(self):
        l_pose, r_pose = self.get_ee_poses()
        print('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAa')
        print(l_pose)
        print(r_pose)

        l_xyz = l_pose.translation
        r_xyz = r_pose.translation

        l_rpy = tf_to_rpy(l_pose)
        r_rpy = tf_to_rpy(r_pose)

        return (l_xyz, l_rpy), (r_xyz, r_rpy)

    def get_ee_poses(self):
        states = np.asarray(self._GetJointStates())
        l_pose = self.l_arm._get_forward_kinematics(states[:7])
        r_pose = self.r_arm._get_forward_kinematics(states[7:])
        return l_pose, r_pose
