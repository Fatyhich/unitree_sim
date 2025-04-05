import numpy as np
import time
from controllers.decartes_controller import DecartesController
from utils.utils import construct_arm_message

class InterpolatingDecartesController(DecartesController):

    def __init__(self, is_in_local=False, network_interface=None, target_dt = 0.005):
        self.target_dt = target_dt
        DecartesController.__init__(self, is_in_local, network_interface)

    def go_to(self, l_xyzrpy = None, r_xyzrpy = None, shoulder = False, l_elbow_xyz=None, r_elbow_xyz=None, dt=0.02):
        cur_point = self._GetJointStates()

        (l_target_q, _), (r_target_q, _) = self.inverse_kinematics(
            l_xyzrpy,
            r_xyzrpy,
            shoulder,
            l_elbow_xyz,
            r_elbow_xyz
        )

        target_q = np.concatenate((l_target_q, r_target_q))

        # get number of interpolation steps
        n_steps = int(dt / self.target_dt)
        # interpolate
        q_values = np.linspace(cur_point, target_q, num=n_steps, endpoint=True)

        # variable that tracks how long and iteration took
        real_dt = 0

        for q in q_values:
            if real_dt > self.target_dt:
                real_dt -= self.target_dt
                continue
            # record current time
            iter_start = time.time()

            # create msg
            msg = construct_arm_message(q)
            self.ExecuteCommand(msg)

            # get end time
            iter_end = time.time()

            real_dt = iter_end - iter_start
            to_sleep = np.clip(self.target_dt - real_dt, 0, self.target_dt)
            time.sleep(to_sleep)
            

        return super().go_to(l_xyzrpy, r_xyzrpy, shoulder, l_elbow_xyz, r_elbow_xyz, dt)