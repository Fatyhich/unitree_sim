from csv_parser import Parser
import pinocchio as pin
from getch import getch
from split_decartes_controller import DecartesController
import argparse

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
    csv_parser.parse_trajectory_file(transform=False)

    controller = DecartesController(network_interface=args.network_interface, is_in_local=args.local)

    counter = 0
    for line_num, line in enumerate(csv_parser):
        wrist_pos = line['wrist_position']
        writst_rot = line['wrist_orientation']

        print('target: ')
        print('xyz: ', wrist_pos)
        print('rpy: ', writst_rot)
        print('press any key to exec')
        (l_xyz, l_rpy), (r_xyz, r_rpy) = controller.get_ee_xyzrpy()
        getch()
        
        controller.go_to(
            l_pos=wrist_pos,
            l_rpy=writst_rot,
            r_pos=r_xyz,
            r_rpy=r_rpy
        )
        (l_xyz, l_rpy), (r_xyz, r_rpy) = controller.get_ee_xyzrpy()
        print('l pose:')
        print('xyz: ', l_xyz)
        print('rpy: ', l_rpy)
        print('r pose:')
        print('xyz: ', r_xyz)
        print('rpy: ', r_rpy)
        print('---------------')
        





if __name__ == '__main__':
    main()