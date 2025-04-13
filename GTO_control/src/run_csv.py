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
from utils.safety_monitoring import SafetyMonitor
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

    target_dt = 0.02

    # max iterations in line
    max_lines = sum(1 for _ in enumerate(csv_parser))
    real_times = np.zeros(max_lines, float)
    target_times = np.zeros_like(real_times)

    RMSE = 0
    while True:
        l_init_elbow_xyz = csv_parser.trajectory_data['elbow_positions'][0]
        l_init_pose = (
            csv_parser.trajectory_data['wrist_positions'][0],
            csv_parser.trajectory_data['wrist_orientations'][0]
        )

        # print('init point: ')
        # print_formatted_target(
        #     l_init_pose[0],
        #     l_init_pose[1],
        #     l_init_elbow_xyz
        # )
        print('GOING HOME')
        go_home(controller, total_time=5)

        print('GOING TO INITIAL POINT', end='\n')
        controller.go_to(
            l_xyzrpy=l_init_pose,
            l_elbow_xyz=l_init_elbow_xyz,
            dt=0.5,
            shoulder=True
        )
        skip_counter = 0
        real_times *= 0
        target_times *= 0
        cur_line = 0
        
        positions = np.zeros((max_lines, 3), float)
        rotations = np.zeros((max_lines, 3), float)
        print('STARTING')
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

            (l_xyz, l_rpy), _ = controller.get_ee_xyzrpy()
            positions[line_num] = l_xyz
            rotations[line_num] = l_rpy

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
        # calculate rmse
        wrist_positions = csv_parser.trajectory_data['wrist_positions']
        pos_error = positions[1:] - wrist_positions[:-1]
        pos_error = pos_error * pos_error
        pos_error = np.sum(pos_error) / pos_error.size
        pos_error = np.sqrt(pos_error)
        print('POSITION RMSE: ', pos_error)
        wrist_orientations = csv_parser.trajectory_data['wrist_orientations']
        rot_error = rotations[1:] - wrist_orientations[:-1]
        rot_error = rot_error * rot_error
        rot_error = np.sum(rot_error) / rot_error.size
        rot_error = np.sqrt(rot_error)
        print('ROTATION RMSE: ', rot_error)
        print('-----------------------')
        print()
        # go_home(controller, total_time=3)
        

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
    viz = None
    logger = None

    try:
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
        smooth_bringup(controller, time=3, dt=0.01)
        # go_home(controller)

        # create visualizer if needed
        if args.visual:
            viz = GlobalVisualizer(cmd_topic='rt/lowcmd')
    
        if not args.no_log:
            logger:LoggerVisuals = LoggerVisuals(
                command_topic='rt/lowcmd'
            )
        

        basic_csv_run(controller, csv_parser)
    except KeyboardInterrupt:
        print("SIGINT received,  returning to home and saving...")
    except RuntimeError:
        print('RUNTIME ERROR, PROBABLY SAFETY VIOLATION')
    except Exception as e:
        print(e)
        raise e
    finally:
        print()
        if logger is not None:
            logger.skip_updates = True
        go_home(controller)
        if logger is not None:
            logger.dump_data()
        else:
            print('NOT SAVING ANY LOG')
        sys.exit(0)

if __name__ == '__main__':
    main()
