import numpy as np
import pinocchio as pin
import time
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK

def main():
    # Initialize the IK solver with visualization
    arm_ik = G1_29_ArmIK(Unit_Test=True, Visualization=True)

    # Center positions for the circular motion
    left_center = np.array([0.3, 0.3, 0.1])
    right_center = np.array([0.3, -0.3, 0.1])
    radius = 0.1

    # Run animation loop
    try:
        while True:
            # Generate circular motion
            for angle in range(0, 360, 5):
                # Convert angle to radians
                theta = np.radians(angle)
                
                # Calculate new positions
                left_pos = left_center + radius * np.array([np.cos(theta), np.sin(theta), 0])
                right_pos = right_center + radius * np.array([np.cos(theta), -np.sin(theta), 0])

                # Create transformation matrices
                left_target = pin.SE3(
                    pin.Quaternion(1, 0, 0, 0).toRotationMatrix(),
                    left_pos
                )
                right_target = pin.SE3(
                    pin.Quaternion(1, 0, 0, 0).toRotationMatrix(),
                    right_pos
                )

                # Solve IK
                q, tau = arm_ik.solve_ik(
                    left_target.homogeneous,
                    right_target.homogeneous
                )

                # Add small delay to make motion visible
                time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping animation...")

if __name__ == "__main__":
    main()