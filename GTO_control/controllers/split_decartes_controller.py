import numpy as np

from controllers.synchronous_controller import SynchronousController
from kinematics.single_arm_kinematics_with_elbow import SingleArmKinematicsWithElbow

from utils.utils import (
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
        self.l_arm = SingleArmKinematicsWithElbow(is_left=True)
        self.r_arm = SingleArmKinematicsWithElbow(is_left=False)
        print('Decartes Controller Initialized')

    def get_initial_guess(self):
        return np.asarray(self._GetLeftJoints()), np.asarray(self._GetRightJoints())

    def go_to(self, l_xyzrpy:tuple=None, r_xyzrpy:tuple=None, shoulder:bool=False, l_elbow_xyz=None, r_elbow_xyz=None):
        # get initial guess
        l_init_guess, r_init_guess = self.get_initial_guess()

        # set joint targets as current 
        l_target_q = l_init_guess
        r_target_q = r_init_guess

        # calculate ik
        if l_xyzrpy is not None:
            l_target_q = self.l_arm.inverse_kinematics(
                xyz=l_xyzrpy[0],
                rpy=l_xyzrpy[1],
                current_motor_q=l_init_guess,
                from_shoulder=shoulder,
                elbow_xyz=l_elbow_xyz
            )

        if r_xyzrpy is not None:
            r_target_q = self.r_arm.inverse_kinematics(
                xyz=r_xyzrpy[0],
                rpy=r_xyzrpy[1],
                current_motor_q=r_init_guess,
                from_shoulder=shoulder,
                elbow_xyz=r_elbow_xyz
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
