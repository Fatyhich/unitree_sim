import time
import sys
import numpy as np
import pinocchio as pin

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, MotorCmd_

# Import our IK solver
sys.path.append('/home/oversir/projects/avp_teleoperate')  # Adjust this path
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK

# Control parameters
dt = 0.05  # Control loop timestep
running_time = 0.0
 
# Initialize IK solver
arm_ik = G1_29_ArmIK(Unit_Test=False, Visualization=True)

# Define target poses for hands
# These could be adjusted based on your needs
left_target = pin.SE3(
    pin.Quaternion(1, 0, 0, 0).toRotationMatrix(),
    np.array([0.3, 0.3, 0.1])
)

right_target = pin.SE3(
    pin.Quaternion(1, 0, 0, 0).toRotationMatrix(),
    np.array([0.3, -0.3, 0.1])
)

input("Press enter to start")

if __name__ == '__main__':
    # Initialize communication
    if len(sys.argv) < 2:
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, sys.argv[1])

    # Create publisher for low-level commands
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    # Initialize motor commands for all joints
    motor_cmds = [
        MotorCmd_(
            mode=0x01,    # Position mode
            q=0.0,        # Position
            dq=0.0,       # Velocity
            tau=0.0,      # Torque
            kp=50.0,      # Position gain
            kd=3.5,       # Velocity gain
            reserve=0     # Reserve value
        ) for _ in range(35)
    ]

    # Create LowCmd message
    cmd = LowCmd_(
        mode_pr=0x01,      # Position mode
        mode_machine=0x00,  # Normal mode
        motor_cmd=motor_cmds,
        reserve=[0, 0, 0, 0],
        crc=0
    )

    # Get initial joint positions
    current_q = np.zeros(14)  # Only for arm joints
    current_dq = np.zeros(14)

    try:
        while True:
            step_start = time.perf_counter()
            running_time += dt

            # Solve IK for current targets
            sol_q, sol_tau = arm_ik.solve_ik(
                left_wrist=left_target.homogeneous,
                right_wrist=right_target.homogeneous,
                current_lr_arm_motor_q=current_q,
                current_lr_arm_motor_dq=current_dq
            )

            # Update arm joint commands (assuming first 14 motors are for arms)
            for i in range(14):
                cmd.motor_cmd[i].q = sol_q[i]
                cmd.motor_cmd[i].dq = 0.0  # Could use velocity from IK if needed
                cmd.motor_cmd[i].tau = sol_tau[i]
                cmd.motor_cmd[i].kp = 50.0
                cmd.motor_cmd[i].kd = 3.5

            # Store current state for next iteration
            current_q = sol_q
            current_dq = np.zeros_like(sol_q)  # Update if using velocity control

            # Send commands
            pub.Write(cmd)

            # Maintain control loop timing
            time_until_next_step = dt - (time.perf_counter() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    except KeyboardInterrupt:
        print("\nStopping robot control...")
        # Set zero commands before exiting
        for motor in cmd.motor_cmd:
            motor.q = 0.0
            motor.dq = 0.0
            motor.tau = 0.0
        pub.Write(cmd)
