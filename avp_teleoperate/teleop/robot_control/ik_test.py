import sys
sys.path.append('/opt/openrobots/lib/python3.8/site-packages')

import numpy as np
import pinocchio as pin
from robot_arm_ik import G1_29_ArmIK

def main():
    # Initialize the IK solver
    # Unit_Test=True means it will look for robot models in the test path
    # Visualization=True means it will open a 3D viewer to show the robot
    arm_ik = G1_29_ArmIK(Unit_Test=True, Visualization=True)

    # Create target poses for left and right hands
    # Each pose is a 4x4 homogeneous transformation matrix
    left_target = pin.SE3(
        pin.Quaternion(1, 0, 0, 0).toRotationMatrix(),  # Rotation matrix (identity in this case)
        np.array([0.3, 0.3, 0.1])  # Position [x, y, z]
    )

    right_target = pin.SE3(
        pin.Quaternion(1, 0, 0, 0).toRotationMatrix(),
        np.array([0.3, -0.3, 0.1])
    )

    # Solve IK
    # This returns joint angles (q) and joint torques (tau)
    q, tau = arm_ik.solve_ik(
        left_wrist=left_target.homogeneous,    # Convert SE3 to 4x4 matrix
        right_wrist=right_target.homogeneous,
        current_lr_arm_motor_q=None,           # Optional: current joint positions
        current_lr_arm_motor_dq=None           # Optional: current joint velocities
    )

    print("Joint angles:", q)
    print("Joint torques:", tau)

if __name__ == "__main__":
    main()