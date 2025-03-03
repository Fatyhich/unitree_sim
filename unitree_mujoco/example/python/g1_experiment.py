import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

# Simulator configuration
DOMAIN_ID = 1
INTERFACE = "lo"

G1_NUM_MOTOR = 29

Kp = [
    60, 60, 60, 100, 40, 40,      # legs
    60, 60, 60, 100, 40, 40,      # legs
    60, 40, 40,                   # waist
    40, 40, 40, 40,  40, 40, 40,  # arms
    40, 40, 40, 40,  40, 40, 40   # arms
]

Kd = [
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1,              # waist
    1, 1, 1, 1, 1, 1, 1,  # arms
    1, 1, 1, 1, 1, 1, 1   # arms 
]

class G1JointIndex:
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11
    WaistYaw = 12
    WaistRoll = 13        # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistA = 13           # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistPitch = 14       # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistB = 14           # NOTE: INVALID for g1 23dof/29dof with waist locked
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20   # NOTE: INVALID for g1 23dof
    LeftWristYaw = 21     # NOTE: INVALID for g1 23dof
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27  # NOTE: INVALID for g1 23dof
    RightWristYaw = 28    # NOTE: INVALID for g1 23dof

class Mode:
    PR = 0  # Series Control for Pitch/Roll Joints
    AB = 1  # Parallel Control for A/B Joints

class Custom:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.002  # [2ms]
        self.duration_ = 5.0    # [2 s]
        self.counter_ = 0
        self.mode_pr_ = Mode.PR
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  
        self.low_state = None 
        self.update_mode_machine_ = False
        self.crc = CRC()
        self.initialized = False  # Flag to track initialization
        
    def Init(self):
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()
        while result['name']:
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)

        # create publisher #
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

        # Initialize mode machine
        self.update_mode_machine_ = True

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        while self.update_mode_machine_ == False:
            time.sleep(1)

        if self.update_mode_machine_ == True:
            self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg

        if self.update_mode_machine_ == False:
            self.mode_machine_ = self.low_state.mode_machine
            self.update_mode_machine_ = True
        
        self.counter_ +=1
        if (self.counter_ % 500 == 0) :
            self.counter_ = 0
            print(self.low_state.imu_state.rpy)

    def LowCmdWrite(self):
        if self.low_state is None:
            return

        self.time_ += self.control_dt_

        if self.time_ < self.duration_:
            # First stage: smooth initialization to current position
            if not self.initialized:
                ratio = np.clip(self.time_ / (self.duration_ * 0.3), 0.0, 1.0)
                for i in range(G1_NUM_MOTOR):
                    self.low_cmd.motor_cmd[i].mode = 1  # Enable
                    self.low_cmd.motor_cmd[i].tau = 0
                    self.low_cmd.motor_cmd[i].dq = 0
                    self.low_cmd.motor_cmd[i].kp = Kp[i]
                    self.low_cmd.motor_cmd[i].kd = Kd[i]
                    current_pos = self.low_state.motor_state[i].q
                    self.low_cmd.motor_cmd[i].q = current_pos * ratio
                
                if ratio >= 1.0:
                    self.initialized = True
                    self.time_ = 0  # Reset time for next phase
            
            # Second stage: move to target position
            else:
                ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
                
                # Keep all joints at their current position except right arm
                # for i in range(G1_NUM_MOTOR):
                #     self.low_cmd.motor_cmd[i].mode = 1
                #     self.low_cmd.motor_cmd[i].tau = 0
                #     self.low_cmd.motor_cmd[i].dq = 0
                #     self.low_cmd.motor_cmd[i].kp = Kp[i]
                #     self.low_cmd.motor_cmd[i].kd = Kd[i]
                #     self.low_cmd.motor_cmd[i].q = self.low_state.motor_state[i].q

                # Smoothly move right arm to target position
                start_shoulder_pitch = self.low_state.motor_state[G1JointIndex.RightShoulderPitch].q
                target_shoulder_pitch = -np.pi/2  # Raised position

                # self.low_cmd.motor_cmd[G1JointIndex.RightShoulderPitch].dq = 0
                self.low_cmd.motor_cmd[G1JointIndex.RightShoulderPitch].q = (
                    start_shoulder_pitch * (1.0 - ratio) + target_shoulder_pitch * ratio
                )

            self.low_cmd.mode_pr = Mode.PR
            self.low_cmd.mode_machine = self.mode_machine_
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.lowcmd_publisher_.Write(self.low_cmd)

if __name__ == '__main__':
    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    print("This program will raise the right arm to a straight position.")
    input("Press Enter to continue...")

    if len(sys.argv) < 2:
    #If no network card is input, use the simulated domain id and the local network card
        ChannelFactoryInitialize(1, "lo")
    else:
    # Otherwise, use the specified network card
        ChannelFactoryInitialize(0, sys.argv[1])
    
    # Initialize with simulator configuration
    # ChannelFactoryInitialize(DOMAIN_ID, INTERFACE)

    custom = Custom()
    custom.Init()
    custom.Start()

    # Keep the program running
    while True:        
        time.sleep(1)