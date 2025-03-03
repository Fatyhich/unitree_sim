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

# Number of fingers/joints in the hand
NUM_FINGERS = 12  # 6 for each hand (right and left)

class HandJointOrder:
    """
    Joint order for each hand according to documentation:
    Right Hand: [pinky, ring, middle, index, thumb_bend, thumb_rotation] (0-5)
    Left Hand:  [pinky, ring, middle, index, thumb_bend, thumb_rotation] (6-11)
    """
    # Right Hand
    RIGHT_PINKY = 0
    RIGHT_RING = 1
    RIGHT_MIDDLE = 2
    RIGHT_INDEX = 3
    RIGHT_THUMB_BEND = 4
    RIGHT_THUMB_ROTATION = 5
    
    # Left Hand
    LEFT_PINKY = 6
    LEFT_RING = 7
    LEFT_MIDDLE = 8
    LEFT_INDEX = 9
    LEFT_THUMB_BEND = 10
    LEFT_THUMB_ROTATION = 11

class HandController:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.002  # [2ms]
        self.duration_ = 2.0      # [2s] for each gesture
        self.counter_ = 0
        self.mode_pr_ = 0  # PR mode
        self.mode_machine_ = 0
        self.hand_cmd = unitree_hg_msg_dds__LowCmd_()
        self.hand_state = None
        self.initialized = False
        self.update_mode_machine_ = False
        self.crc = CRC()

        # Predefined hand positions (normalized 0-1 range)
        self.gestures = {
            "open": np.ones(6),
            "close": np.zeros(6),
            "half": np.ones(6) * 0.5,
            "pinch": np.array([0, 0, 0, 0.3, 0.3, 0.5]),  # Only thumb and index slightly open
            "victory": np.array([0, 0, 0.8, 0.8, 0, 0.5]), # Middle and index fingers up
            "thumbs_up": np.array([0, 0, 0, 0, 1.0, 0.8])  # Only thumb up
        }

    def Init(self):
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()
        while result['name']:
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)

        # Create publisher for hand commands
        self.handcmd_publisher_ = ChannelPublisher("rt/inspire/cmd", LowCmd_)
        self.handcmd_publisher_.Init()

        # Create subscriber for hand state
        self.handstate_subscriber = ChannelSubscriber("rt/inspire/state", LowState_)
        self.handstate_subscriber.Init(self.HandStateHandler, 10)

        # Initialize mode machine
        self.update_mode_machine_ = True

    def Start(self):
        self.handCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.HandCmdWrite, name="hand_control"
        )
        while not self.update_mode_machine_:
            time.sleep(1)

        if self.update_mode_machine_:
            self.handCmdWriteThreadPtr.Start()

    def HandStateHandler(self, msg: LowState_):
        self.hand_state = msg
        
        if not self.update_mode_machine_:
            self.mode_machine_ = self.hand_state.mode_machine
            self.update_mode_machine_ = True
        
        self.counter_ += 1
        if self.counter_ % 500 == 0:
            self.counter_ = 0
            # Print current finger positions for right hand
            right_angles = [self.hand_state.motor_state[i].q for i in range(6)]
            print(f"Right hand angles: {right_angles}")

    def set_hand_position(self, right_angles, left_angles=None, ratio=1.0):
        """
        Set hand position with normalized angles (0-1 range)
        right_angles: array of 6 values for right hand
        left_angles: array of 6 values for left hand (optional)
        ratio: interpolation ratio for smooth movement
        """
        print('set_hand_position')
        if left_angles is None:
            left_angles = right_angles  # Mirror right hand by default

        # Ensure arrays are numpy arrays
        right_angles = np.array(right_angles)
        left_angles = np.array(left_angles)

        # Apply to right hand (first 6 joints)
        for i in range(6):
            current = self.hand_state.motor_state[i].q if self.hand_state else 0.0
            target = right_angles[i]
            self.hand_cmd.motor_cmd[i].mode = 1  # Enable
            self.hand_cmd.motor_cmd[i].q = current * (1.0 - ratio) + target * ratio
            self.hand_cmd.motor_cmd[i].tau = 0.0
            self.hand_cmd.motor_cmd[i].dq = 0.0
            self.hand_cmd.motor_cmd[i].kp = 0.0
            self.hand_cmd.motor_cmd[i].kd = 0.0

        # Apply to left hand (last 6 joints)
        # for i in range(6):
        #     current = self.hand_state.motor_state[i+6].q if self.hand_state else 0.0
        #     target = left_angles[i]
        #     self.hand_cmd.motor_cmd[i+6].mode = 1  # Enable
        #     self.hand_cmd.motor_cmd[i+6].q = current * (1.0 - ratio) + target * ratio
        #     self.hand_cmd.motor_cmd[i+6].tau = 0.0
        #     self.hand_cmd.motor_cmd[i+6].dq = 0.0
        #     self.hand_cmd.motor_cmd[i+6].kp = 0.0
        #     self.hand_cmd.motor_cmd[i+6].kd = 0.0

    def apply_gesture(self, gesture_name, ratio=1.0, mirror=True):
        """Apply a predefined gesture to both hands"""
        print('apply_gesture')
        if gesture_name in self.gestures:
            right_angles = self.gestures[gesture_name]
            if mirror:
                self.set_hand_position(right_angles, right_angles, ratio)
            else:
                self.set_hand_position(right_angles, self.gestures["open"], ratio)
        else:
            print(f"Unknown gesture: {gesture_name}")

    def HandCmdWrite(self):
        print('hand command write')
        if not self.hand_state:
            return

        self.time_ += self.control_dt_

        # First stage: Initialize to open position
        if not self.initialized:
            ratio = np.clip(self.time_ / (self.duration_ * 0.5), 0.0, 1.0)
            self.apply_gesture("open", ratio)
            
            if ratio >= 1.0:
                self.initialized = True
                self.time_ = 0  # Reset time for gesture sequence

        # Second stage: Cycle through gestures
        else:
            gesture_duration = self.duration_ * 2  # Time for each gesture
            gestures = ["open", "close", "pinch", "victory", "thumbs_up", "half"]
            current_time = self.time_ % (gesture_duration * len(gestures))
            gesture_index = int(current_time / gesture_duration)
            ratio = np.clip((current_time % gesture_duration) / gesture_duration, 0.0, 1.0)
            
            # Apply the current gesture
            self.apply_gesture(gestures[gesture_index], ratio)

        # Set mode and CRC
        self.hand_cmd.mode_pr = self.mode_pr_
        self.hand_cmd.mode_machine = self.mode_machine_
        self.hand_cmd.crc = self.crc.Crc(self.hand_cmd)
        
        # Publish command
        self.handcmd_publisher_.Write(self.hand_cmd)

if __name__ == '__main__':
    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    print("This program will demonstrate different hand gestures using the Inspire RH56DFX hand:")
    print("1. Open hand")
    print("2. Close hand")
    print("3. Pinch (thumb and index)")
    print("4. Victory sign")
    print("5. Thumbs up")
    print("6. Half open")
    input("Press Enter to continue...")

    if len(sys.argv) < 2:
    #If no network card is input, use the simulated domain id and the local network card
        ChannelFactoryInitialize(1, "lo")
    else:
    # Otherwise, use the specified network card
        ChannelFactoryInitialize(0, sys.argv[1])

    controller = HandController()
    controller.Init()
    controller.Start()

    # Keep the program running
    while True:
        time.sleep(1) 