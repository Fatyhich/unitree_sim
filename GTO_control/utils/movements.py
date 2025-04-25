import numpy as np
from time import sleep
from controllers.synchronous_controller import SynchronousController
from utils.arm_definitions import (
    G1JointArmIndex,
    G1JointIndex
    )
from utils.utils import construct_arm_message   

def smooth_drop(controller, time=3.0, dt=0.01):
    times = np.arange(0, time, dt)
    for t in times:
        percentage = np.clip(t / time, 0., 1.)
        values = np.zeros(7)
        # pos = controller.low_state.motor_state[joint].q
        message = {}
        for idx, joint in enumerate(G1JointIndex):
            pos = 0.
            # vel = (new_pos - pos) / dt
            # pos = new_pos
            message[joint] = (pos, 0, 0)

def smooth_bringup(controller, time=2.0, dt= 0.01):
    times = np.arange(0, time * 1.1, dt)
    for t in times:
        percentage = np.clip(t / time, 0., 1.)
        values = np.zeros(7)
        # pos = controller.low_state.motor_state[joint].q
        message = {}
        for idx, joint in enumerate(G1JointIndex):
            pos = (1. - percentage) * controller.low_state.motor_state[joint].q
            # vel = (new_pos - pos) / dt
            # pos = new_pos
            message[joint] = (pos, 0, 0)


        controller.ExecuteCommand(message)
        sleep(dt)

def test_sine(
        controller,
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
            joint_idx : (pos, vel, 0)
        }
        print(pos)
        controller.ExecuteCommand(command)
        sleep(dt)

def go_home(
        controller:SynchronousController,
        total_time=6,
        dt=0.01
    ):
    print('GOING HOME')
    
    n_steps = int(total_time / dt)
    controller._UPDATES_LOCKED = False
    current_states = controller._GetJointStates()
    # disable updates so that locking does not happen again
    controller._UPDATES_LOCKED = True
    
    total_targets = np.zeros((len(current_states), n_steps))
    controller._CONTROL_LOCKED = False
    for idx in range(len(current_states)):
        total_targets[idx] = np.linspace(current_states[idx], 0, n_steps)

    total_targets = total_targets.T

    for idx in range(len(total_targets)):
        msg = construct_arm_message(
            total_targets[idx]
        )
        controller.ExecuteCommand(msg)
        sleep(dt)