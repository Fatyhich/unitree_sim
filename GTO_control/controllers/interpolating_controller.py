import numpy as np
import time
from controllers.decartes_controller import DecartesController
from utils.utils import construct_arm_message

class InterpolatingDecartesController(DecartesController):

    def __init__(self, is_in_local=False, network_interface=None, target_dt = 0.005):
        self.target_dt = target_dt
        DecartesController.__init__(self, is_in_local, network_interface)

    def go_to(
            self,
            l_xyzrpy = None,
            r_xyzrpy = None,
            shoulder = False,
            l_elbow_xyz=None,
            r_elbow_xyz=None,
            dt=0.02,
            do_skips:bool=False
        ):
        call_time = time.time()
        cur_q = self._GetJointStates()

        (l_target_q, _), (r_target_q, _) = self.inverse_kinematics(
            l_xyzrpy=l_xyzrpy,
            r_xyzrpy=r_xyzrpy,
            shoulder=shoulder,
            l_elbow_xyz=l_elbow_xyz,
            r_elbow_xyz=r_elbow_xyz
        )
        
        target_q = np.concatenate((l_target_q, r_target_q))
        # if debug:
        #     print('fk: ')
        #     print(self.l_arm.get_ee_pose(l_target_q), False)
        #     print('target: ', l_target_q)

        n_steps = int(dt / self.target_dt)
        dq = (target_q - cur_q) / n_steps

        execution_start = time.time()
        time_left = dt - (execution_start - call_time)
        new_target_dt = time_left / n_steps
        real_dt = 0
        for step_idx in range(n_steps):
            cur_q += dq
            if real_dt > new_target_dt and do_skips:
                real_dt -= new_target_dt
                continue
            # record current time
            iter_start = time.time()

            # create msg
            msg = construct_arm_message(cur_q)
            self.ExecuteCommand(msg)

            # get end time
            iter_end = time.time()

            real_dt = iter_end - iter_start
            to_sleep = np.clip(new_target_dt - real_dt, 0, new_target_dt)
            time.sleep(to_sleep)

        # send the target point just because i do not trust it
        # msg = construct_arm_message(target_q)
        # self.ExecuteCommand(msg)
            

        return super().go_to(l_xyzrpy, r_xyzrpy, shoulder, l_elbow_xyz, r_elbow_xyz, dt)