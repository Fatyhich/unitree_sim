from utils.csv_parser import Parser
from time import sleep
import pinocchio as pin
from getch import getch
from controllers.split_decartes_controller import DecartesController
import argparse
from utils.utils import smooth_bringup
from kinematics.kinematics_visualizer import KinematicsVisualizer
import matplotlib.pyplot as plt

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('input_file', help='CSV file to read')
    parser.add_argument(
        '-n', 
        '--num', 
        help='Number of lines to read', 
        type=int, 
        default=1
        )
    parser.add_argument(
        '-ni', 
        '--network-interface', 
        help='Network interface for control output.'
        )
    parser.add_argument(
        '-l',
        '--local',
        help='Specifies if robot is run in simulation',
        action='store_true'
        )
    parser.add_argument(
        '-uc', '--use-control', 
        help='If true, a controller will be created that will execute everything.', 
        action='store_true'
    )
    parser.add_argument(
        '-v', '--visual',
        help='If True, will also display kinematics of the robot',
        action='store_true'
    )
    args = parser.parse_args()
    return args

def main():
    args = parse_args()
    csv_parser = Parser(args.input_file)
    csv_parser.parse_trajectory_file()

    if args.use_control:
        controller = DecartesController(network_interface=args.network_interface, is_in_local=args.local)
        smooth_bringup(controller)
        _, (r_xyz, r_rpy) = controller.get_ee_xyzrpy()

    if args.visual:
        viz = KinematicsVisualizer()

    counter = 0
    for line_num, line in enumerate(csv_parser):
        wrist_pos = line['wrist_position']
        wrist_rot = line['wrist_orientation']
        elbow_pos = line['elbow_position']

        print('-----TARGET-----')
        print(' xyz: ', wrist_pos)
        print(' rpy: ', wrist_rot)
        print('----------------')
        print()
        # print('press any key to exec')

        if args.visual:
            # display kinematics
            viz.inverse_kinematics(
                (wrist_pos, wrist_rot),
                l_elbow_target=elbow_pos
                # (r_xyz, r_rpy)
            )

            # wait for key
            while not plt.waitforbuttonpress(0):
                pass
        
        if args.use_control:
            controller.go_to(
                l_xyzrpy=(wrist_pos, wrist_rot),
                shoulder=True
            )
            sleep(0.02)

            # print current position
            # (l_xyz, l_rpy), _ = controller.get_ee_xyzrpy()
            # print('-----CURRENT-----')
            # print(' xyz: ', l_xyz)
            # print(' rpy: ', l_rpy)
            # print('-----------------')
            # print()


if __name__ == '__main__':
    main()