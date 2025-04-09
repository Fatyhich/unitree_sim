from getch import getch
import numpy as np
import matplotlib.pyplot as plt
import cv2
import argparse
import sys

from time import time
from time import sleep

import pinocchio as pin
from utils import utils
from utils.csv_parser import Parser
from controllers.decartes_controller import DecartesController
from controllers.interpolating_controller import InterpolatingDecartesController
from kinematics.kinematics_visualizer import KinematicsVisualizer
from utils.logger_visuals import LoggerVisuals
from utils.arm_definitions import G1JointIndex
from utils.global_meshcat import GlobalVisualizer
from utils.movements import (
    go_home,
    smooth_bringup
)

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
    parser.add_argument(
        '--interp', 
        help='If true will interpolate data. NOTE: DO NOT USE IT, IT IS VERY DANGEROUS. WIP.',
        action='store_true'
    )
    parser.add_argument(
        '--no-log', help='If true, no log will be dumped after program finishes.',
        action='store_true'
    )
    args = parser.parse_args()
    return args

def print_formatted_target(wrist_pos, wrist_rot, elbow_pos):
        print('-----TARGET-----')
        print(' xyz: ', wrist_pos)
        print(' rpy: ', wrist_rot)
        print(' elbow: ', elbow_pos)
        print('----------------')
        print()

def execute_trajectory(
        wrist_xyz,
        wrist_rpy,
        elbow_pos,
        dt,
        controller:DecartesController=None,
        visualizer:KinematicsVisualizer=None,
        is_left:bool=True
    ):

    if len(wrist_xyz) != len(wrist_rpy) or len(wrist_xyz) != len(elbow_pos):
        print('ERROR: all components must have equal lengths')
        exit(1)

    iteration_time = 0
    for idx in range(len(wrist_xyz)):
        start = time()
        if controller is not None:
            controller.go_to(
                l_xyzrpy=(wrist_xyz[idx], wrist_rpy[idx]),
                l_elbow_xyz=elbow_pos[idx],
                shoulder=True
            )
        if visualizer is not None:
            print('SORRY, NOT IMPLEMENTED YET')
        end = time()
        if cv2.waitKey(max(dt - (end - start), 0)) == ' ':
            utils.smooth_bringup(controller)

def basic_csv_run(controller:DecartesController, csv_parser:Parser):
    l_init_elbow_xyz = csv_parser.trajectory_data['elbow_positions'][0]

    l_init_pose = (
        csv_parser.trajectory_data['wrist_positions'][0],
        csv_parser.trajectory_data['wrist_orientations'][0]
    )

    # controller.go_to(
    #     l_xyzrpy=l_init_pose,
    #     l_elbow_xyz=l_init_elbow_xyz,
    #     shoulder=True,
    #     dt=2
    #     )

    # record starting time
    target_dt = 0.02

    # max iterations in line
    max_lines = sum(1 for _ in enumerate(csv_parser))
    real_times = np.zeros(max_lines, float)
    target_times = np.zeros_like(real_times)
    while True:
        l_init_elbow_xyz = csv_parser.trajectory_data['elbow_positions'][0]
        l_init_pose = (
            csv_parser.trajectory_data['wrist_positions'][0],
            csv_parser.trajectory_data['wrist_orientations'][0]
        )

        print('GOING TO INITIAL POINT')
        print('')
        controller.go_to(
            l_xyzrpy=l_init_pose,
            l_elbow_xyz=l_init_elbow_xyz,
            dt=5,
            shoulder=True
        )

        skip_counter = 0
        print('STARTING')
        real_times *= 0
        target_times *= 0
        cur_line = 0
        for line_num, line in enumerate(csv_parser):
            cur_line += 1
            if cur_line > 20:
                pass

            # get iteration start time
            iter_start = time()

            # get target positions
            wrist_pos = line['wrist_position']
            wrist_rot = line['wrist_orientation']
            elbow_pos = line['elbow_position']

            # save target time for current command 
            target_times[line_num] = line['time']
            # save time to check that is is sane
            real_times[line_num] = time()

            # make sure that times align properly
            if (real_times[line_num] - real_times[0]) > line['time']:
                skip_counter += 1
                continue

            # send command 
            controller.go_to(
                l_xyzrpy=(wrist_pos, wrist_rot),
                l_elbow_xyz=elbow_pos,
                shoulder=True,
                dt=target_dt
            )


            # get iteration end time
            iter_end = time()
            # calculate iteration time
            iter_time = iter_end - iter_start
            # compensate iteration
            sleep_for = np.clip(target_dt - iter_time, 0, target_dt)
            sleep(sleep_for)


        print('END')
        print('TARGET TIME: ', target_times[-1] - target_times[0])
        print('FULL TIME: ', real_times[-1] - real_times[0])
        print(f'SKIPPED: {skip_counter} total, {skip_counter/max_lines} relative')
        # utils.go_home(controller, dt=0.01, total_time=3)
        print('-----------------------')
        print()
        go_home(controller, total_time=3)
        # getch()
        # utils.go_home(controller)



def main():
    # parse arguments
    args = parse_args()
    use_control = args.use_control

    # create a parser and read the file
    csv_parser = Parser(args.input_file)
    csv_parser.parse_trajectory_file()

    controller = None
    # create control if needed
    if use_control:
        if args.interp:
            controller = InterpolatingDecartesController(
                network_interface=args.network_interface,
                is_in_local=args.local
            )
        else:
            controller = DecartesController(
                network_interface=args.network_interface, 
                is_in_local=args.local
            )
        controller.smooth_bringup()
        # bring ip up smoothly
        # utils.smooth_bringup(controller)
        # utils.go_home(controller)
        # also save xyzrpy for right hand
        _, (r_xyz, r_rpy) = controller.get_ee_xyzrpy()

    viz = None
    # create visualizer if needed
    if args.visual:
        viz = GlobalVisualizer(cmd_topic='rt/lowcmd')
    
    # get current poses if needed

    logger:LoggerVisuals = LoggerVisuals(
        command_topic='rt/lowcmd'
    )

    try:
        basic_csv_run(controller, csv_parser)
    except KeyboardInterrupt:
        print("SIGINT received,  returning to home and saving...")
        logger.skip_updates = True
        go_home(controller)
        if not args.no_log:
            logger.dump_data()
        else:
            print('NOT SAVING ANY LOG')
        sys.exit(0)  # Exit gracefully

if __name__ == '__main__':
    main()



    # # XXX THIS CODE IS UNSAFE
    # # XXX DO NOT RUN ON THE REAL ROBOT 
    # # XXX UNLESS YOU FIXED IT
    # if False and args.interp:
    #     # shift arrays
    #     all_states[:, :-1] = all_states[:, 1:]
    #     times[:-1] = times[1:]

    #     # fill most recent values
    #     all_states[0, -1] = line['wrist_position']
    #     all_states[1, -1] = line['wrist_orientation']
    #     all_states[2, -1] = line['elbow_position']
    #     times[-1] = line['time']

    #     print('prev: ', all_states[0, 1])
    #     print('next: ', all_states[0, -1])

    #     # interpolate    
    #     wrist_xyz_interp, wrist_rpy_interp, elbow_xyz_interp, new_times, new_dt = spline_interpolation(
    #         wrist_positions=all_states[0],
    #         wrist_orientations=all_states[1],
    #         elbow_positions=all_states[2],
    #         times=times
    #         )

    #     plt.plot(
    #         new_times,
    #         wrist_xyz_interp[:, 0]
    #     )
    #     plt.plot(
    #         new_times,
    #         wrist_xyz_interp[:, 1]
    #     )
    #     plt.pause(0.1)
    #     while not plt.waitforbuttonpress(0):
    #         pass

    #     # execute_trajectory(
    #     #     wrist_xyz_interp,
    #     #     wrist_rpy_interp,
    #     #     elbow_xyz_interp,
    #     #     new_dt,
    #     #     controller,
    #     #     viz
    #     # )

    # if args.visual:
    #     # display kinematics
    #     viz.inverse_kinematics(
    #         (wrist_pos, wrist_rot),
    #         l_elbow_target=elbow_pos,
    #         origin='shoulder'
    #         # (r_xyz, r_rpy)
    #     )

    #     # wait for key
    #     while not plt.waitforbuttonpress(0):
    #         pass
        