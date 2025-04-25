import numpy as np
import time

from utils.arm_definitions import G1JointIndex
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize # dds
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_                                 # idl
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC
from utils.arm_definitions import G1JointLeftArmIndex, G1JointRightArmIndex
from utils.utils import construct_arm_message

Kp = [
    60, 60, 60, 100, 40, 40,      # legs
    60, 60, 60, 100, 40, 40,      # legs
    60, 40, 40,                   # waist
    40, 40, 40, 40,  40, 40, 40,  # arms
    40, 40, 40, 40,  40, 40, 40,  # arms
    0                             # not used joint
]

Kd = [
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1,              # waist
    1, 1, 1, 1, 1, 1, 1,  # arms
    1, 1, 1, 1, 1, 1, 1,  # arms 
    0                     # not used joint
]

class SynchronousController:

    def __init__(
            self, 
            network_interface:str=None, 
            is_in_local:bool=False, 
            command_topic="rt/arm_sdk",
            do_safety_checks:bool=True,
            safety_limit=3.
        ):
        """Initializes a controller

        Args:
            network_interface (str, optional): 
                Specifies which hardware interface
                is used to transfer commands. 
                SELECTED HARDWARE INTERFACE IS RECOMMENDED TO HAVE IP 192.168.123.99
                Defaults to None.
            is_in_local (bool, optional): 
                Used to indicate that robot runs in a simulation. 
                Used only for ChannelFactoryInitialize(1, "lo").
                Defaults to False.
            command_topic (str, optional): 
                Specifies topic on which commands will be published.
                Must be "rt/arm_sdk" or "rt/low_cmd".
            do_safety_checks (bool, optional):
                If set to False, will not conduct any safety checks for joint
                velocity dq.
                Defaults to True.
            safety_limit (float, optional):
                Maximal joint velocity that can be achieved without safety
                violations.
                Defaults to 3.

        """

        self.do_safety_checks = do_safety_checks
        self.safety_limit = safety_limit
        self.command_topic = command_topic

        # set pid controls
        self.kp = 40.
        self.kd = 1.5
        self.waist_kp = 250.
        self.waist_kd = 3.5
        self.dq = 0.
        self.tau_ff = 0.
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  
        self.low_state = None 
        self.first_update_low_state = False
        self.crc = CRC()
        self._CONTROL_LOCKED = False
        self._UPDATES_LOCKED = False

        # set joitns
        self.arm_joints = [
            G1JointIndex.LeftShoulderPitch,  G1JointIndex.LeftShoulderRoll,
            G1JointIndex.LeftShoulderYaw,    G1JointIndex.LeftElbow,
            G1JointIndex.LeftWristRoll,      G1JointIndex.LeftWristPitch,
            G1JointIndex.LeftWristYaw,
            G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll,
            G1JointIndex.RightShoulderYaw,   G1JointIndex.RightElbow,
            G1JointIndex.RightWristRoll,     G1JointIndex.RightWristPitch,
            G1JointIndex.RightWristYaw,
            G1JointIndex.WaistYaw,
            G1JointIndex.WaistRoll,
            G1JointIndex.WaistPitch
        ]

        if is_in_local:
            self.log('initializing for local')
            ChannelFactoryInitialize(1, "lo")
            self.command_topic = "rt/lowcmd"
        elif network_interface is not None:
            self.log(f'initalizing with {network_interface} as interface')
            ChannelFactoryInitialize(0, network_interface)
        else:
            self.log('initalizing without interface')
            ChannelFactoryInitialize(0)

        self.__Init()


    def __Init(self):

        # create publisher #
        self.arm_sdk_publisher = ChannelPublisher(self.command_topic, LowCmd_)
        self.arm_sdk_publisher.Init()

        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

        self.log('Controller Topics Initialized')

        self.log('Waiting to init position...')
        # initialize joint position #
        while self.first_update_low_state == False:
            time.sleep(1)

        self.log("Joint State Initialized")

    def lock(self):
        if self._CONTROL_LOCKED or self._UPDATES_LOCKED:   
            return
        self._UPDATES_LOCKED = True
        self._CONTROL_LOCKED = True
        print('CONTROLLER LOCKED\n-----------------')

    def log(self, msg):
        print(f'[{__name__}] ', msg)

    def check_safety(self):
        for idx, joint in enumerate(G1JointIndex):
            cur_vel = self.low_state.motor_state[joint].dq
            if abs(cur_vel) > self.safety_limit:
                self.lock()
                print(f'SAFETY VIOLATED ON JOINT {idx}(', joint, ')', sep='')
                print(f'CURRENT LIMIT: {self.safety_limit}')
                print(f'VELOCITY VIOL: {cur_vel}')
    
    def LowStateHandler(self, msg: LowState_) -> None:
        if self._UPDATES_LOCKED:
            return
        self.low_state = msg
        if self.do_safety_checks:
            self.check_safety()

        if self.first_update_low_state == False:
            self.first_update_low_state = True
        
    def __SetCommand(self, targets:dict):
        """Contstructs command for all specified joints.

        Args:
            targets (dict): Keys are joint indeces to write, 
                values are triplets of target values (q, dq, torq_ff)
        """
        # enable arm sdk
        self.low_cmd.motor_cmd[G1JointIndex.NotUsedJoint0].q = 1 # 1:Enable arm_sdk, 0:Disable arm_sdk

        # create command
        for idx, joint in enumerate(G1JointIndex):
            self.low_cmd.motor_cmd[joint].mode = 1
            # set tau to zero
            self.low_cmd.motor_cmd[joint].tau = 0.

            # if joint is in message, add command
            if joint in targets.keys():
                # add q
                self.low_cmd.motor_cmd[joint].q = targets[joint][0]
                # add dq
                self.low_cmd.motor_cmd[joint].dq = targets[joint][1]
                # add torq
                self.low_cmd.motor_cmd[joint].tau = targets[joint][2]

            # otherwise set command to zero
            else:
                self.low_cmd.motor_cmd[joint].q = 0.
                self.low_cmd.motor_cmd[joint].dq = 0.
            
            # set all joints kp and kd to default
            # to avoid if, waist kp and kd are set later
            self.low_cmd.motor_cmd[joint].kp =  Kp[idx]
            self.low_cmd.motor_cmd[joint].kd =  Kd[idx]
        
        self.low_cmd.motor_cmd[G1JointIndex.NotUsedJoint0].q = 1 # 1:Enable arm_sdk, 0:Disable arm_sdk
        
        # # set kp and kd for waist roll
        # self.low_cmd.motor_cmd[G1JointIndex.WaistRoll].kp = self.waist_kp
        # self.low_cmd.motor_cmd[G1JointIndex.WaistRoll].kd = self.waist_kd
        # # and for waist pitch motor
        # self.low_cmd.motor_cmd[G1JointIndex.WaistPitch].kp = self.waist_kp
        # self.low_cmd.motor_cmd[G1JointIndex.WaistPitch].kd = self.waist_kd
        # # and for waist yaw
        # self.low_cmd.motor_cmd[G1JointIndex.WaistYaw].kp = self.waist_kp
        # self.low_cmd.motor_cmd[G1JointIndex.WaistYaw].kd = self.waist_kd

    def __PublishCommand(self):
        """Sends command to it's topic
        """
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)
        # self.log("Command published")

    def ExecuteCommand(self, targets:dict):
        """Executes given target velocities and positons for given joints.

        Args:
            targets (dict): Keys are joint indeces, 
                values are pairs of joint position and velocity (q, dq)
        """
        if self._CONTROL_LOCKED:
            print('ERROR TRYING TO EXECUTE COMMAND ON LOCKED CONTROLLER')
            raise RuntimeError
        self.__SetCommand(targets)
        self.__PublishCommand()

    def _GetLeftJoints(self):
        result = np.zeros(len(G1JointLeftArmIndex))
        for idx, joint in  enumerate(G1JointLeftArmIndex):
            result[idx] = self.low_state.motor_state[joint].q
        return result

    def _GetRightJoints(self):
        result = np.zeros(len(G1JointRightArmIndex))
        for idx, joint in enumerate(G1JointRightArmIndex):
            result[idx] = self.low_state.motor_state[joint].q
        return result
    
    def _GetJointStates(self):
        left = self._GetLeftJoints()
        right = self._GetRightJoints()
        return np.concatenate((left, right))
    
    def _GetLeftVelocities(self):
        result = np.zeros(len(G1JointLeftArmIndex))
        for idx, joint in G1JointLeftArmIndex:
            result[idx] = self.low_state.motor_state[joint].dq
        return result
    
    def _GetRightVelocities(self):
        result = []
        for joint in G1JointRightArmIndex:
            result.append(self.low_state.motor_state[joint.dq])
        return result

def main():
    controller = SynchronousController(is_in_local=True)

if __name__ == '__main__':
    main()


