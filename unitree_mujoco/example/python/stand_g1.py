import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, MotorCmd_

# Initial standing position for humanoid (35 joints)
stand_up_joint_pos = np.zeros(35, dtype=float)
# Example values for some main joints (adjust these based on your robot's specifications)
# Legs (0-11): L_LEG and R_LEG joints
stand_up_joint_pos[0:12] = 0.0  # Hip, knee, ankle joints
# Waist (12-14): WAIST_YAW, WAIST_ROLL, WAIST_PITCH
stand_up_joint_pos[12:15] = 0.0
# Left arm (15-21): L_SHOULDER to L_WRIST joints
stand_up_joint_pos[15:22] = 0.0
# Right arm (22-28): R_SHOULDER to R_WRIST joints
stand_up_joint_pos[22:29] = 0.0
# Additional joints (29-34): Any remaining joints
stand_up_joint_pos[29:35] = 0.0

# Crouching position
stand_down_joint_pos = np.zeros(35, dtype=float)
# Modify these values for a safe crouching position
stand_down_joint_pos[0:12] = 0.2  # Slightly bent legs
stand_down_joint_pos[12:15] = 0.0  # Straight waist
stand_down_joint_pos[15:29] = 0.0  # Arms in neutral position
stand_down_joint_pos[29:35] = 0.0  # Additional joints in neutral position

dt = 1/ 200
running_time = 0.0

input("Press enter to start")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, sys.argv[1])

    # Create a publisher to publish the data
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    # Initialize motor commands array with all required arguments
    motor_cmds = [
        MotorCmd_(
            mode=0x01,    # Position mode
            q=0.0,        # Position
            dq=0.0,       # Velocity
            tau=0.0,      # Torque
            kp=0.0,       # Position gain
            kd=0.0,       # Velocity gain
            reserve=0     # Reserve value
        ) for _ in range(35)
    ]

    # Create LowCmd with all required arguments
    cmd = LowCmd_(
        mode_pr=0x01,      # Position mode
        mode_machine=0x00,  # Normal mode
        motor_cmd=motor_cmds,
        reserve=[0, 0, 0, 0],  # 4 reserve values
        crc=0
    )

    while True:
        step_start = time.perf_counter()
        running_time += dt

        if (running_time < 3.0):
            # Stand up in first 3 seconds
            phase = np.tanh(running_time / 1.2)
            for i in range(35):
                cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (
                    1 - phase) * stand_down_joint_pos[i]
                cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.5
                cmd.motor_cmd[i].tau = 0.0
        else:
            # Then stand down
            phase = np.tanh((running_time - 3.0) / 1.2)
            for i in range(35):
                cmd.motor_cmd[i].q = phase * stand_down_joint_pos[i] + (
                    1 - phase) * stand_up_joint_pos[i]
                cmd.motor_cmd[i].kp = 50.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.5
                cmd.motor_cmd[i].tau = 0.0

        pub.Write(cmd)

        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
