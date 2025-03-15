from gto_csv_parser import Parser
import pinocchio as pin
from getch import getch
from decartes_controller import DecartesController
import argparse

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('input_file', help='CSV file to read')
    parser.add_argument('-n', '--num', help='Number of lines to read', type=int, default=1)
    parser.add_argument('-ni', '--network-interface', help='Network interface for control output.')
    args = parser.parse_args()
    return args

def main():
    args = parse_args()
    csv_parser = Parser(args.input_file)
    csv_parser.parse_trajectory_file()

    controller = DecartesController(network_interface=args.network_interface)

    counter = 0
    for line_num, line in enumerate(csv_parser):
        wrist_pos = line['wrist_position']
        writst_rot = line['wrist_orientation']

        t_pos = pin.SE3(pin.rpy.rpyToMatrix(writst_rot), wrist_pos)
        t_id = controller.arm.reduced_robot.model.getJointId("left_shoulder_pitch_joint1")
        controller.arm.reduced_robot

        print('target: ')
        print('xyz: ', wrist_pos)
        print('rpy: ', writst_rot)
        print('press any key to exec')
        l_pos, r_pos = controller.get_ee_poses()
        r_xyz = r_pos.translation
        r_rot = pin.rpy.matrixToRpy(r_pos.rotation)
        getch()
        
        controller.go_to(
            l_pos=wrist_pos,
            l_rpy=writst_rot,
            r_pos=r_xyz,
            r_rpy=r_rot
        )
        l_pos, r_pos = controller.get_ee_poses()
        r_xyz = r_pos.translation
        r_rot = pin.rpy.matrixToRpy(r_pos.rotation)
        l_xyz = l_pos.translation
        l_rot = pin.rpy.matrixToRpy(l_pos.rotation)
        print('l pose:')
        print('xyz: ', l_xyz)
        print('rpy: ', l_rot)
        print('r pose:')
        print('xyz: ', r_xyz)
        print('rpy: ', r_rot)
        print('---------------')
        





if __name__ == '__main__':
    main()