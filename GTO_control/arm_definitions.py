from enum import IntEnum

class G1JointArmIndex(IntEnum):
    # Left arm
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristyaw = 21

    # Right arm
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28

class G1JointLeftArmIndex(IntEnum):
    # Left arm
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristyaw = 21

class G1JointRightArmIndex(IntEnum):
    # Right arm
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28

class G1JointIndex(IntEnum):
    # Left leg
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleRoll = 5

    # Right leg
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11

    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14

    # Left arm
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21

    # Right arm
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28
    
    # not used
    NotUsedJoint0 = 29
    # NotUsedJoint1 = 30
    # NotUsedJoint2 = 31
    # NotUsedJoint3 = 32
    # NotUsedJoint4 = 33
    # NotUsedJoint5 = 34

ALL_JOINTS = [
    "left_hip_pitch_joint" ,
    "left_hip_roll_joint" ,
    "left_hip_yaw_joint" ,

    "left_knee_joint" ,

    "left_ankle_pitch_joint" ,
    "left_ankle_roll_joint" ,

    "right_hip_pitch_joint" ,
    "right_hip_roll_joint" ,
    "right_hip_yaw_joint" ,

    "right_knee_joint" ,

    "right_ankle_pitch_joint" ,
    "right_ankle_roll_joint" ,

    "waist_yaw_joint" ,
    "waist_roll_joint" ,
    "waist_pitch_joint" ,

    "left_shoulder_pitch_joint" ,
    "left_shoulder_roll_joint" ,
    "left_shoulder_yaw_joint" ,

    "left_elbow_joint" ,

    "left_wrist_roll_joint" ,
    "left_wrist_pitch_joint" ,
    "left_wrist_yaw_joint" ,

    "left_hand_thumb_0_joint" ,
    "left_hand_thumb_1_joint" ,
    "left_hand_thumb_2_joint" ,
    "left_hand_middle_0_joint" ,
    "left_hand_middle_1_joint" ,
    "left_hand_index_0_joint" ,
    "left_hand_index_1_joint" ,

    "right_shoulder_pitch_joint" ,
    "right_shoulder_roll_joint" ,
    "right_shoulder_yaw_joint" ,

    "right_elbow_joint" ,

    "right_wrist_roll_joint" ,
    "right_wrist_pitch_joint" ,
    "right_wrist_yaw_joint" ,

    "right_hand_thumb_0_joint" ,
    "right_hand_thumb_1_joint" ,
    "right_hand_thumb_2_joint" ,
    "right_hand_middle_0_joint" ,
    "right_hand_middle_1_joint" ,
    "right_hand_index_0_joint" ,
    "right_hand_index_1_joint" ,
]

LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint" ,
    "left_shoulder_roll_joint" ,
    "left_shoulder_yaw_joint" ,

    "left_elbow_joint" ,

    "left_wrist_roll_joint" ,
    "left_wrist_pitch_joint" ,
    "left_wrist_yaw_joint" ,
]

RIGHT_ARM_JOINTS = [
    "right_shoulder_pitch_joint" ,
    "right_shoulder_roll_joint" ,
    "right_shoulder_yaw_joint" ,

    "right_elbow_joint" ,

    "right_wrist_roll_joint" ,
    "right_wrist_pitch_joint" ,
    "right_wrist_yaw_joint" ,
]

NON_ARM_JOINTS = [
    "left_hip_pitch_joint" ,
    "left_hip_roll_joint" ,
    "left_hip_yaw_joint" ,
    "left_knee_joint" ,
    "left_ankle_pitch_joint" ,
    "left_ankle_roll_joint" ,
    "right_hip_pitch_joint" ,
    "right_hip_roll_joint" ,
    "right_hip_yaw_joint" ,
    "right_knee_joint" ,
    "right_ankle_pitch_joint" ,
    "right_ankle_roll_joint" ,
    "waist_yaw_joint" ,
    "waist_roll_joint" ,
    "waist_pitch_joint" ,

    "left_hand_thumb_0_joint" ,
    "left_hand_thumb_1_joint" ,
    "left_hand_thumb_2_joint" ,
    "left_hand_middle_0_joint" ,
    "left_hand_middle_1_joint" ,
    "left_hand_index_0_joint" ,
    "left_hand_index_1_joint" ,

    "right_hand_thumb_0_joint" ,
    "right_hand_thumb_1_joint" ,
    "right_hand_thumb_2_joint" ,
    "right_hand_index_0_joint" ,
    "right_hand_index_1_joint" ,
    "right_hand_middle_0_joint",
    "right_hand_middle_1_joint"
]