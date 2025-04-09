import numpy as np
from time import sleep
import matplotlib.pyplot as plt
import argparse
from controllers.synchronous_controller import SynchronousController
from utils.arm_definitions import G1JointArmIndex
from utils.arm_definitions import G1JointIndex

def process_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-ni', '--network-interface', 
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
    controller = SynchronousController(
        network_interface=args.network_interface, 
        is_in_local=args.local
    )
    controller.smooth_bringup()
    controller.smooth_drop()
    # smooth_bringup(controller)
    # test_sine(controller, dt=0.01, joint_idx=G1JointArmIndex.LeftWristyaw)
    # smooth_bringup(controller)

if __name__ == '__main__':
    main()
        
