import numpy as np

from synchronous_controller import SynchronousController
from arm_kinematics import ArmKinematics

from utils import log, construct_arm_message


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

    def go_to(self, l_tf_homo, r_tf_homo):

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
        
    def get_ee_poses(self):
        states = np.asarray(self._GetJointStates())
        l_pose, r_pose = self.arm._get_forward_kinematics(states)
        return l_pose, r_pose
