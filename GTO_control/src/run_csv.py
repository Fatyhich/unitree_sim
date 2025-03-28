from utils.csv_parser import Parser
import numpy as np
from time import time
from time import sleep
import pinocchio as pin
from getch import getch
from controllers.decartes_controller import DecartesController
from controllers.synchronous_controller import SynchronousController
import argparse
from utils.utils import smooth_bringup
from kinematics.kinematics_visualizer import KinematicsVisualizer
import matplotlib.pyplot as plt

def do_hand_plots(times, to_plot, fig=None, ax=None):
    if fig is None or ax is None:
        fig, ax = plt.subplots(2, 4)
    for idx in range(7):
        for time, data in zip(times, to_plot):
            ax[int(idx / 4), idx % 4].plot(time, data[idx])
            ax[int(idx / 4), idx % 4].grid()
    plt.show()
    return fig, ax

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('input_file', help='CSV file to read')
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

def get_all_data():
    pass

def main():
    args = parse_args()
    csv_parser = Parser(args.input_file)
    csv_parser.parse_trajectory_file()

    if args.use_control:
        controller = DecartesController(
            network_interface=args.network_interface, 
            is_in_local=args.local, 
            do_logging=True
        )
        smooth_bringup(controller)
        _, (r_xyz, r_rpy) = controller.get_ee_xyzrpy()

    if args.visual:
        viz = KinematicsVisualizer()

    start = time()

    for line_num, line in enumerate(csv_parser):
        wrist_pos = line['wrist_position']
        wrist_rot = line['wrist_orientation']
        elbow_pos = line['elbow_position']

        print('-----TARGET-----')
        print(' xyz: ', wrist_pos)
        print(' rpy: ', wrist_rot)
        print(' elbow: ', elbow_pos)
        print('----------------')
        print()
        # print('press any key to exec')

        if args.visual:
            # display kinematics
            viz.inverse_kinematics(
                (wrist_pos, wrist_rot),
                l_elbow_target=elbow_pos,
                origin='shoulder'
                # (r_xyz, r_rpy)
            )

            # wait for key
            while not plt.waitforbuttonpress(0):
                pass
        
        if args.use_control:
            controller.go_to(
                l_xyzrpy=(wrist_pos, wrist_rot),
                shoulder=True,
                l_elbow_xyz=elbow_pos,
                dt=0.02
            )
            sleep(0.02)

            # print current position
            # (l_xyz, l_rpy), _ = controller.get_ee_xyzrpy()
            # print('-----CURRENT-----')
            # print(' xyz: ', l_xyz)
            # print(' rpy: ', l_rpy)
            # print('-----------------')
            # print()
        print('TARGET TIME: ', line['time'])
    end = time()
    print('FULL TIME: ', end - start)

if __name__ == '__main__':
    main()