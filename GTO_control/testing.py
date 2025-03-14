import numpy as np
from time import sleep
import matplotlib.pyplot as plt
import argparse
from synchronous_controller import SynchronousController
from arm_definitions import G1JointArmIndex
from arm_definitions import G1JointIndex

def create_controller(output_interface=None, in_local=False):
    controller = SynchronousController(
        output_interface=output_interface,
        in_local=in_local
        )
    
    return controller

def smooth_bringup(controller:SynchronousController, time=3.0, dt= 0.02):
    times = np.arange(0, time * 1.1, dt)
    for t in times:
        percentage = np.clip(t / time, 0., 1.)
        message = {}
        # pos = controller.low_state.motor_state[joint].q
        for joint in G1JointIndex:
            pos = (1. - percentage) * controller.low_state.motor_state[joint].q
            # vel = (new_pos - pos) / dt
            # pos = new_pos
            message[joint] = (pos, 0)

        controller.ExecuteCommand(message)
        sleep(dt)

def test_sine(
        controller:SynchronousController, 
        total_time=10, 
        dt=0.02, 
        omega=np.pi/6,
        joint_idx=G1JointArmIndex.LeftWristRoll
        ):
    # generate velocities
    times = np.arange(0, total_time, dt)
    sine_positions = np.sin(times * omega)
    cosine_velocities = np.diff(sine_positions) / dt
    for pos, vel in zip(sine_positions[:-1], cosine_velocities):
        command = {
            joint_idx : (pos, vel)
        }
        print(pos)
        controller.ExecuteCommand(command)
        sleep(dt)

def process_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('-ni', '--network-interface', 
                        help='Specifies which interface to use for real robot testing',
                        required=False
                        )
    parser.add_argument(
        '-l', '--local',
        help='Specifies whether robot is being run in a simulator.',
        action='store_true',
        default=False
    )
    args = parser.parse_args()
    return args
    

def main():
    args = process_arguments()
    controller = create_controller(
        output_interface=args.network_interface,
        in_local=args.local
    )
    smooth_bringup(controller)
    test_sine(controller, dt=0.01)

if __name__ == '__main__':
    main()
        
