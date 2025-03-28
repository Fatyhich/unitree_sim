import numpy as np
import time
import pickle

from utils.arm_definitions import G1JointIndex

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize # dds
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_                                 # idl
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

class JointLogger():

    def __init__(
            self,
            network_interface:str=None, 
            is_in_local:bool=False,
            command_topic='rt/arm_sdk',
            do_dump:bool=True
            ):

        self.command_topic = command_topic
        self.is_in_local = is_in_local
        self.do_dump = do_dump

        self.crc = CRC()

        self.skip_updates = False

        # create fields for real data
        self.real_time = []
        self.real_q = []
        self.real_dq = []
        self.real_ddq = []

        # create fields for commands
        self.control_time = []
        self.target_q = []
        self.target_dq = []
        self.target_torq = []

        self.data_dict = {
            "real_q" : self.real_q,
            "real_dq" : self.real_dq,
            "real_ddq" : self.real_ddq,
            "real_times" : self.real_time,
            "target_q" : self.target_q,
            "target_dq" : self.target_dq,
            "target_torque" : self.target_torq,
            "control_times" : self.control_time
        }

        # initalize all topics
        self.create_subs()


    def create_subs(self):
        # create sub for joint states
        self.state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.state_sub.Init(self.add_states, 10)

        # create sub for commands
        self.cmd_sub = ChannelSubscriber(self.command_topic, LowCmd_)
        self.cmd_sub.Init(self.add_cmd, 10)

    def add_states(self, msg:LowState_):
        if self.skip_updates:
            return

        self.real_time.append(time.time())
        cur_q = np.zeros(len(G1JointIndex))
        cur_dq = np.zeros(len(G1JointIndex))
        cur_ddq= np.zeros(len(G1JointIndex))

        for idx, joint in enumerate(G1JointIndex):
            cur_q[joint] = msg.motor_state[joint].q
            cur_dq[joint] = msg.motor_state[joint].dq
            cur_ddq[joint] = msg.motor_state[joint].ddq

        self.real_q.append(cur_q)
        self.real_dq.append(cur_dq)
        self.real_ddq.append(cur_ddq)

    def add_cmd(self, msg:LowCmd_):
        if self.skip_updates:
            return

        self.control_time.append(time.time())
        cur_q = np.zeros(len(G1JointIndex))
        cur_dq = np.zeros(len(G1JointIndex))
        cur_tau = np.zeros(len(G1JointIndex))

        for idx, joint in enumerate(G1JointIndex):
            cur_q[joint] = msg.motor_cmd[joint].q
            cur_dq[joint] = msg.motor_cmd[joint].dq
            cur_tau[joint] = msg.motor_cmd[joint].tau
        
        self.target_q.append(cur_q)
        self.target_dq.append(cur_dq)
        self.target_torq.append(cur_tau)

    def get_data_for_joints(
        self,
        joint_ids:np.ndarray,
        desired_data:list
    ):
        """Returns dictionary with all requested data field for all requested joints.

        Args:
            joint_ids (np.ndarray): Array of joint indeces that you want to check. Must be joint index from arm_definitions.
            desired_data (list): List of data fields that you want to check.
                
                Possible options:
                    "real_q" - real joint states.
                    "real_dq" - real joint velocities.
                    "real_ddq" - real joint accelerations.
                    "real_times" - times at which joints were logged.
                    "target_q" - joint states that were passed as targets.
                    "target_dq" - joint velocities that were passed as targets.
                    "target_torque" - joint torques(tau feed forward) that were passed to control.
                    "control_times" - times at which controls were recorded.
                }

        Returns:
            dict: The keys in the result are the requested data fields.
            Values are arrays of data field for requested joints.
            Example where we queried real_q for joints [17, 10]:
            get_data_for_joints([17, 10], ["real_q"])
            will return:
            {
                "real_q" : [real_q_for_joint_17, real_q_for_joint_10]
            }.

        """

        result = {}

        self.skip_updates = True
        for data_field in desired_data:
            cur_data = np.asarray(self.data_dict[data_field]).T
            result[data_field] = cur_data[joint_ids]
        self.skip_updates = False
        return result
    
    def dump_data(self, filename:str=f"logger_dump_{time.time()}.pkl"):
        with open(filename, 'wb') as dump_file:
            pickle.dump(self.data_dict, dump_file)
        

    def __del__(self):
        if self.do_dump:
            self.dump_data()

if __name__ == '__main__':
    log = JointLogger("enp195s0f3u1")