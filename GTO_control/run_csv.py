from csv_parser import Parser
import pinocchio as pin
from getch import getch
from split_decartes_controller import DecartesController
import argparse
from utils import smooth_bringup
from kinematics_visualizer import KinematicsVisualizer

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('input_file', help='CSV file to read')
    parser.add_argument('-n', '--num', help='Number of lines to read', type=int, default=1)
    parser.add_argument('-ni', '--network-interface', help='Network interface for control output.')
    parser.add_argument('-l', '--local', help='Specifies if robot is run in simulation', action='store_true')
    args = parser.parse_args()
    return args

def main():
    args = parse_args()
    csv_parser = Parser(args.input_file)
    # position = (0, 0.137926, 0.290497)
    # quaternion = (0, 0, 0, 1)
    # csv_parser.set_transform_matrix(position, quaternion=quaternion)
    csv_parser.parse_trajectory_file(transform=False, inverse=False)

    # controller = DecartesController(network_interface=args.network_interface, is_in_local=args.local)
    # smooth_bringup(controller)

    viz = KinematicsVisualizer()

    counter = 0
    for line_num, line in enumerate(csv_parser):
        wrist_pos = line['wrist_position'] * 0.412
        wrist_rot = line['wrist_orientation']

        print('target: ')
        print('xyz: ', wrist_pos)
        print('rpy: ', wrist_rot)
        print('press any key to exec')
        # (l_xyz, l_rpy), (r_xyz, r_rpy) = controller.get_ee_xyzrpy()
        # print('left: ')
        # print(l_xyz)
        # print(l_rpy)
        # print('right: ')
        # print(r_xyz)
        # print(r_rpy)

        viz.inverse_kinematics_shoulder(
            (wrist_pos, wrist_rot)
            # (r_xyz, r_rpy)
            )

        getch()
        
        # controller.go_to(
        #     l_pos=wrist_pos,
        #     l_rpy=wrist_rot,
        #     r_pos=r_xyz,
        #     r_rpy=r_rpy
        # )
        # (l_xyz, l_rpy), (r_xyz, r_rpy) = controller.get_ee_xyzrpy()
        # print('l pose:')
        # print('xyz: ', l_xyz)
        # print('rpy: ', l_rpy)
        # print('r pose:')
        # print('xyz: ', r_xyz)
        # print('rpy: ', r_rpy)
        # print('---------------')
        





if __name__ == '__main__':
    main()