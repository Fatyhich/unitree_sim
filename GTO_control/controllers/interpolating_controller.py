import numpy as np
import time
from controllers.decartes_controller import DecartesController
from utils.utils import construct_arm_message
from utils.arm_definitions import G1JointArmIndex

class InterpolatingDecartesController(DecartesController):

    def __init__(
            self,
            is_in_local=False,
            network_interface=None,
            target_dt = 0.005, 
            do_safety_checks:bool=True,
            safety_limit:float=3.
            ):
        self.target_dt = target_dt
        self.last_target_joints = np.zeros(len(G1JointArmIndex), float)
        DecartesController.__init__(
            self,
            is_in_local,
            network_interface,
            do_safety_checks=do_safety_checks,
            safety_limit=safety_limit
            )

    def go_to(
            self,
            l_xyzrpy = None,
            r_xyzrpy = None,
            shoulder = False,
            l_elbow_xyz=None,
            r_elbow_xyz=None,
            dt=0.02,
            use_velocity:bool=False,
            do_skips:bool=False
        ):
        call_time = time.time()

        (l_target_q, l_tauff), (r_target_q, r_tauff) = self.inverse_kinematics(
            l_xyzrpy=l_xyzrpy,
            r_xyzrpy=r_xyzrpy,
            shoulder=shoulder,
            l_elbow_xyz=l_elbow_xyz,
            r_elbow_xyz=r_elbow_xyz
        )
        target_q = np.concatenate((l_target_q, r_target_q))
        target_tauff = np.concatenate((l_tauff, r_tauff))

        n_steps = int(dt / self.target_dt)
        dq = (target_q - self.last_target_joints) / n_steps
        velocitites = dq / self.target_dt

        execution_start = time.time()
        time_left = dt - (execution_start - call_time)
        new_target_dt = time_left / n_steps
        real_dt = 0
        cur_q = self.last_target_joints
        for step_idx in range(n_steps):
            cur_q += dq
            if real_dt > new_target_dt and do_skips:
                real_dt -= new_target_dt
                continue
            # record current time
            iter_start = time.time()

            # create msg
            msg = construct_arm_message(
                joint_states=cur_q,
                joint_velocities=velocitites if use_velocity else None,
                torq_ff=target_tauff
            )

            self.ExecuteCommand(msg)

            # get end time
            iter_end = time.time()

            real_dt = iter_end - iter_start
            to_sleep = np.clip(new_target_dt - real_dt, 0, new_target_dt)
            time.sleep(to_sleep)

        self.last_target_joints = target_q
        # send the target point just because i do not trust it
        # msg = construct_arm_message(target_q)
        # self.ExecuteCommand(msg)
            
