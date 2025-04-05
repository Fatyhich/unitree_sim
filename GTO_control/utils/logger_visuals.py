import numpy as np
from utils.logger import JointLogger
import matplotlib.pyplot as plt

class LoggerVisuals(JointLogger):
    def __init__(
            self, 
            command_topic='rt/arm_sdk', 
            dump_on_death = True,
            local_load_file:str=None
        ):
        """A Visualisation wrapper around the base logger.

        Args:
            command_topic (str, optional): Topic from which control commands are logged. Defaults to 'rt/arm_sdk'.
            dump_on_death (bool, optional): If true, will save its data dict as .pkl file. Defaults to True.
            local_load_file (str, optional): If not none will load a dumped data dict file. Defaults to None.
        """
        super().__init__(command_topic, dump_on_death=dump_on_death, local_load_file=local_load_file)

    def compare_joint_states(self, joint_idx, ax=None):
        """Draws joint states and joint state control in time according to logs.

        Args:
            joint_idx (int): Joint that you want to check. Must be in G1ArmJointIndex
            ax (plottable, optional): Used to plot in subplots' ax[i] objects. 
                If None, will plot using matplotlib.pyplot.plot().
                Defaults to None.
        """
        if ax is None:
            fig, ax = plt.subplots()

        self.skip_updates = True
        real_q = np.asarray(self.real_q).T[joint_idx]
        target_q = np.asarray(self.target_q).T[joint_idx]

        ax.plot(np.array(self.real_time)[1:] - self.real_time[0], real_q)
        ax.plot(np.array(self.control_time)[1:] - self.control_time[0], target_q)
        self.skip_updates = False
        ax.grid()
        ax.set_xlabel("time, s")
        ax.set_ylabel("q, rad")
        ax.legend(
            ["real", "target"]
        )
        if ax is None:
            plt.show()

    def compare_joint_velocities(self, joint_idx, ax=None):
        """Draws joint velocities and joint velcoity control in time according to logs.

        Args:
            joint_idx (int): Joint that you want to check. Must be in G1ArmJointIndex
            ax (plottable, optional): Used to plot in subplots' ax[i] objects. 
                If None, will plot using matplotlib.pyplot.plot().
                Defaults to None.
        """
        if ax is None:
            fig, ax = plt.figure()

        self.skip_updates = True
        real_dq = np.asarray(self.real_dq).T[joint_idx]
        target_dq = np.asarray(self.target_dq).T[joint_idx]

        ax.plot(np.array(self.real_time)[1:]- self.real_time[0], real_dq)
        ax.plot(np.array(self.control_time)[1:] - self.control_time[0], target_dq)
        self.skip_updates = False
        ax.grid()
        ax.set_xlabel("time, s")
        ax.set_ylabel("dq, rad/s")
        ax.legend(
            ["real", "target"]
        )
        if ax is None:
            plt.show()
    
    def plot_full_motion(self, joint_idx, wait_for_key_to_close:bool=True):
        """Creates 3 subplots:
            - Comparison of real joint states and joint state controls.
            - Comparison of real joint velocities and their control.
            - Feed-forwad torque.

        Args:
            joint_idx (int): Index of the joint you want to inspect.
                Must be in G1ArmJointIndex.
        """
        fig, ax = plt.subplots(3, 1)
        self.compare_joint_states(joint_idx, ax[0])
        self.compare_joint_velocities(joint_idx, ax[1])

        torque = np.asarray(self.target_torq).T[joint_idx]

        ax[2].plot(np.array(self.control_time)[1:] - self.control_time[0], torque)
        ax[2].grid()
        ax[2].set_xlabel("time, s")
        ax[2].set_ylabel("torque")
        ax[2].legend(
            ["torque_ff"]
        )
        fig.suptitle(f'JOINT {joint_idx}')
        plt.pause(0.1)
    
    def plot_targets(self, joint_idx):
        fig, ax = plt.subplots(2, 1)

        control_times = np.array(self.control_time)[1:] - self.control_time[0]
        real_times = np.array(self.real_time)[1:] - self.real_time[0]

        target_q = np.asarray(self.target_q).T[joint_idx]
        target_dq = np.asarray(self.target_dq).T[joint_idx]

        ax[0].plot(control_times, target_q)
        ax[0].grid()

        ax[1].plot(control_times, target_dq)
        ax[1].grid()

        fig.suptitle(f'JOINT {joint_idx}')
        plt.pause(0.1)