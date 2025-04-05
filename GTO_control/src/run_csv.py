from getch import getch
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import cv2
import argparse
import signal
import sys

from time import time
from time import sleep

import pinocchio as pin
from utils import utils
from utils.csv_parser import Parser
from controllers.decartes_controller import DecartesController
from kinematics.kinematics_visualizer import KinematicsVisualizer
from utils.logger_visuals import LoggerVisuals
from utils.arm_definitions import G1JointIndex
from utils.global_meshcat import GlobalVisualizer

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

def spline_interpolation(wrist_positions, wrist_orientations, elbow_positions, times, target_dt=0.002):
    """Perform cubic spline interpolation between trajectory points.

    Args:
        wrist_positions (list or np.ndarray): List of wrist positions for interpolation.
        wrist_orientations (list or np.ndarray): List of wrist orientations for interpolation.
        elbow_positions (list or np.ndarray): List of elbow positions for interpolation.
        times (list or np.ndarray): List of time points corresponding to the positions.
        target_dt (float): Desired time step for interpolation.

    Returns:
        tuple: A tuple containing:
            - interpolated_wrist_positions (np.ndarray): Interpolated wrist positions.
            - interpolated_wrist_orientations (np.ndarray): Interpolated wrist orientations.
            - interpolated_elbow_positions (np.ndarray): Interpolated elbow positions.
            - dt (float): Time difference of the interpolation (target_dt).
    """

    # Create interpolating time array
    interp_times = np.arange(times[0], times[-1], target_dt)

    # Create spline functions for positions, orientations, and elbow positions
    wrist_spline = CubicSpline(times, wrist_positions, axis=0)
    orientation_spline = CubicSpline(times, wrist_orientations, axis=0)
    elbow_spline = CubicSpline(times, elbow_positions, axis=0)

    # Generate interpolated data
    interpolated_wrist_positions = wrist_spline(interp_times)
    interpolated_wrist_orientations = orientation_spline(interp_times)
    interpolated_elbow_positions = elbow_spline(interp_times)

    # Return the interpolated data and dt
    return (
        interpolated_wrist_positions,
        interpolated_wrist_orientations,
        interpolated_elbow_positions,
        interp_times,
        target_dt
    )

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
    # record starting time
    target_dt = 0.005

    # max iterations in line
    max_lines = sum(1 for _ in enumerate(csv_parser))
    real_times = np.zeros(max_lines, float)
    target_times = np.zeros_like(real_times)
    while True:
        skip_counter = 0
        print('STARTING')
        for line_num, line in enumerate(csv_parser):
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
                shoulder=True,
                l_elbow_xyz=elbow_pos
                # dt=0.02
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
        utils.go_home(controller, dt=0.01)
        print('-----------------------')
        print()
        getch()



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
        controller = DecartesController(
            network_interface=args.network_interface, 
            is_in_local=args.local
        )
        # bring ip up smoothly
        # utils.smooth_bringup(controller)
        utils.go_home(controller)
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
        utils.go_home(controller)
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
        