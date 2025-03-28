import numpy as np
from utils.logger import JointLogger
import matplotlib.pyplot as plt

class LoggerVisuals(JointLogger):
    def __init__(
            self, 
            network_interface = None, 
            is_in_local = False, 
            command_topic='rt/arm_sdk', 
            do_dump = True
        ):
        super().__init__(network_interface, is_in_local, command_topic, do_dump)

    def compare_joint_states(self, joint_idx):
        self.skip_updates = True
        real_q = np.asarray(self.real_q).T[joint_idx]
        target_q = np.asarray(self.target_q).T[joint_idx]

        plt.plot(self.real_time, real_q)
        plt.plot(self.control_time, target_q)
        self.skip_updates = False
        plt.grid()
        plt.xlabel("time, s")
        plt.ylabel("q, rad")
        plt.legend(
            ["real", "target"]
        )
        plt.show()

    def compare_joint_velocities(self, joint_idx):
        self.skip_updates = True
        real_dq = np.asarray(self.real_dq).T[joint_idx]
        target_dq = np.asarray(self.target_dq).T[joint_idx]

        plt.plot(self.real_time, real_dq)
        plt.plot(self.control_time, target_dq)
        self.skip_updates = False
        plt.grid()
        plt.xlabel("time, s")
        plt.ylabel("dq, rad/s")
        plt.legend(
            ["real", "target"]
        )
        plt.show()