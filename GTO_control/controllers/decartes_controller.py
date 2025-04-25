import numpy as np

from controllers.synchronous_controller import SynchronousController
from kinematics.single_arm_kinematics_with_elbow import SingleArmKinematicsWithElbow

from utils.utils import (
    log, 
    construct_arm_message,
    SE3_from_xyz_rpy,
    SE3_to_xyzrpy,
    tf_to_rpy
)


class DecartesController(SynchronousController):
    
    def __init__(
            self, 
            is_in_local=False, 
            network_interface=None
            ):

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

    def inverse_kinematics(
            self,
            l_xyzrpy:tuple=None,
            r_xyzrpy:tuple=None,
            shoulder:bool=False,
            l_elbow_xyz=None,
            r_elbow_xyz=None
        ):
        # get initial guess
        l_init_guess, r_init_guess = self.get_initial_guess()

        # set joint targets as current 
        l_target_q = l_init_guess
        r_target_q = r_init_guess

        # initialize torques
        l_torq_ff = np.zeros_like(l_target_q)
        r_torq_ff = np.zeros_like(r_target_q)

        # calculate ik
        if l_xyzrpy is not None:
            l_target_q, l_torq_ff = self.l_arm.inverse_kinematics(
                xyz=l_xyzrpy[0],
                rpy=l_xyzrpy[1],
                current_motor_q=l_init_guess,
                from_shoulder=shoulder,
                elbow_xyz=l_elbow_xyz
            )

        if r_xyzrpy is not None:
            r_target_q, r_torq_ff = self.r_arm.inverse_kinematics(
                xyz=r_xyzrpy[0],
                rpy=r_xyzrpy[1],
                current_motor_q=r_init_guess,
                from_shoulder=shoulder,
                elbow_xyz=r_elbow_xyz
            )

        return (l_target_q, l_torq_ff), (r_target_q, r_torq_ff)

    def go_to(
            self,
            l_xyzrpy:tuple=None,
            r_xyzrpy:tuple=None,
            shoulder:bool=False,
            l_elbow_xyz=None,
            r_elbow_xyz=None,
            dt:float=0
            ):
        """Moves hands in decartes space.

        Args:
            l_xyzrpy (tuple, optional): Target pose of the left wrist. 
                Must be a tuple of 2 arrays with shape(3,): ([x, y, z], [r, p, y]),
                Defaults to None.
            r_xyzrpy (tuple, optional): Target pose of the right wrist. 
                Must be a tuple of 2 arrays with shape(3,): ([x, y, z], [r, p, y]),
                Defaults to None.
            shoulder (bool, optional): If true, all coordinates are considered to be relative to shoulder joint. 
                Otherwise, they are considered relative to pelvis.
                Defaults to False.
            l_elbow_xyz (_type_, optional): Target xyz of the left elbow. 
                If **None**, elbow will not be considered.
                Defaults to None.
            r_elbow_xyz (_type_, optional): Target xyz of the right elbow. 
                If **None**, elbow will not be considered.
                Defaults to None.
            dt (float, optional): If > 0, will also set joint speed. 
                Defaults to 0.
        """

        (l_target_q, l_torq_ff), (r_target_q, r_torq_ff) = self.inverse_kinematics(
            l_xyzrpy,
            r_xyzrpy,
            shoulder,
            l_elbow_xyz,
            r_elbow_xyz
        )

        if False: # dt > 0:
            l_dq = (l_target_q - self._GetLeftJoints()) / dt
            r_dq = (r_target_q - self._GetRightJoints()) / dt
        else:
            l_dq = np.zeros_like(l_target_q)
            r_dq = np.zeros_like(r_target_q)

        msg = construct_arm_message(
            joint_states=np.concatenate((l_target_q, r_target_q)),
            joint_velocities=np.concatenate((l_dq, r_dq)),
            torq_ff=np.concatenate((l_torq_ff, r_torq_ff))
            )

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
    
    def get_all_poses(self, in_shoulder:bool=False):
        states = np.asarray(self._GetJointStates())
        l_shoulder, l_elbow, l_wrist = self.l_arm.get_arm_points(states[:7], in_shoulder=in_shoulder)
        r_shoulder, r_elbow, r_wrist = self.r_arm.get_arm_points(states[7:], in_shoulder=in_shoulder)

        return (l_shoulder, l_elbow, l_wrist), (r_shoulder, r_shoulder, r_wrist)


