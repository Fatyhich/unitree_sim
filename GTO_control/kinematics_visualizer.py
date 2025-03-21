import matplotlib.pyplot as plt
from time import time
import numpy as np
from single_arm_kinematics import SingleArmKinematics
# from csv_parser import Parser

class KinematicsVisualizer():

    def __init__(self):
        self.l_kinematics = SingleArmKinematics(is_left=True)
        self.r_kinematics = SingleArmKinematics(is_left=False)
        # add plot to draw on
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        # set axis parameters
        self.MAX_DIM = 1
        self.X_CENTER = 0
        self.Y_CENTER = 0
        self.Z_CENTER = 0

        # create arrays for arm poses
        self.l_xyz = None
        self.r_xyz = None
        self.l_joints = np.zeros(7, float)
        self.r_joints = np.zeros(7, float)
        self.l_target = None
        self.r_target = None
        # fill arrays with zero poses
        # and draw them
        self.forward_kinematics(self.l_joints, self.r_joints)
        self.update_arms_viz()

         
    def set_axes(self):
        # set labels
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        # set limits
        self.ax.set_xlim3d([-self.MAX_DIM + self.X_CENTER, self.MAX_DIM + self.X_CENTER])
        self.ax.set_ylim3d([-self.MAX_DIM + self.Y_CENTER, self.MAX_DIM + self.Y_CENTER])
        self.ax.set_zlim3d([-self.MAX_DIM + self.Z_CENTER, self.MAX_DIM + self.Z_CENTER])

    def draw_arm(self, points_xyz, color=[0., 0., 1.], marker='o'):
        n_points = len(points_xyz)
        x = points_xyz[:, 0]
        y = points_xyz[:, 1]
        z = points_xyz[:, 2]
        # create colors to make first points dimmer
        colors = np.zeros((n_points, 3))
        colors[:, 0] = np.linspace(0.5, color[0], n_points, endpoint=True)
        colors[:, 1] = np.linspace(0.5, color[1], n_points, endpoint=True)
        colors[:, 2] = np.linspace(0.5, color[2], n_points, endpoint=True)
        self.ax.scatter(
            xs=x,
            ys=y,
            zs=z,
            c=colors,
            marker=marker
        )

        arrows_x = np.diff(x)
        arrows_y = np.diff(y)
        arrows_z = np.diff(z)
        # draw arrows from shoulder to wrist
        self.ax.quiver(
            x[:-1], 
            y[:-1], 
            z[:-1],
            arrows_x,
            arrows_y,
            arrows_z,
            color=color,
            arrow_length_ratio=0.5
        )

    def draw_target(self, xyz, color=[0, 0, 1], marker='x'):
        if xyz is None:
            return
        self.ax.scatter(
            xs=xyz[0],
            ys=xyz[1],
            zs=xyz[2],
            color=color,
            marker=marker,
            s=70
        )

    def update_arms_viz(self):
        # clear everything
        self.ax.cla()
        # draw arms
        self.draw_arm(self.l_xyz, color=[0, 0, 1])
        self.draw_arm(self.r_xyz, color=[1, 0, 0])
        # draw targets
        self.draw_target(self.l_target, color=[0, 0, 1])
        self.draw_target(self.r_target, color=[1, 0, 0])
        # set axes
        self.set_axes()
        plt.pause(0.01)

    def __get_arm_points_fk(self, left:bool):
        if left:
            shoulder_pos, elbow_pos, ee_pos = self.l_kinematics.get_arm_points(self.l_joints)
        else:
            shoulder_pos, elbow_pos, ee_pos = self.r_kinematics.get_arm_points(self.r_joints)

        return np.array([
            shoulder_pos.translation,
            elbow_pos.translation, 
            ee_pos.translation
        ])

    def forward_kinematics(self, l_joints=None, r_joints=None, update=True):
        if l_joints is None and r_joints is None:
            print('WARN: UPDATING ARMS WITH NO JOINT POS. NOTHIN WILL BE DONE')
            return
        
        if l_joints is not None:
            self.l_joints = l_joints
            self.l_xyz = self.__get_arm_points_fk(left=True)
        
        if r_joints is not None:
            self.r_joints = r_joints
            self.r_xyz = self.__get_arm_points_fk(left=False)
        if update:
            self.update_arms_viz()
        
    def inverse_kinematics_pelvis(self, l_xyzrpy:tuple=None, r_xyzrpy:tuple=None, update=True):
        """Solves and visualizes kinematics based on target positin in pelvis frame of reference

        Args:
            l_xyzrpy (tuple, optional): 
                Tuple of np.ndarray with shapes (3,). 
                First is xyz, second is rpy. 
                Defaults to None.
            r_xyzrpy (tuple, optional): 
                Tuple of no.ndarray with shapes (3,). 
                First is xyz, second is rpy. 
                Represents target position for right wrist. 
                Defaults to None.
            update (bool, optional): If true, will also update visualization. Defaults to True.
        """
        if l_xyzrpy is None and r_xyzrpy is None:
            print('WARN: UPDATING ARMS WITH NO TARGET POS. NOTHINg WILL BE DONE')

        if l_xyzrpy is not None:
            self.l_target = l_xyzrpy[0]
            l_ik = self.l_kinematics.inverse_kinematics(
                xyz=l_xyzrpy[0],
                rpy=l_xyzrpy[1],
                current_motor_q=self.l_joints
            )
            self.l_joints = l_ik

        if r_xyzrpy is not None:
            self.r_target = r_xyzrpy[0]
            r_ik = self.r_kinematics.inverse_kinematics(
                xyz = r_xyzrpy[0],
                rpy = r_xyzrpy[1],
                current_motor_q=self.r_joints
            )
            self.r_joints = r_ik

        self.forward_kinematics(
            l_joints=self.l_joints,
            r_joints=self.r_joints,
            update=False
        )

        if update:
            self.update_arms_viz()

    def inverse_kinematics_shoulder(self, l_xyzrpy:tuple=None, r_xyzrpy:tuple=None, update=True):
        """Solves and visualizes kinematics based on target positin in shoulder frame of reference

        Args:
            l_xyzrpy (tuple, optional): 
                Tuple of np.ndarray with shapes (3,). 
                First is xyz, second is rpy. 
                Defaults to None.
            r_xyzrpy (tuple, optional): 
                Tuple of no.ndarray with shapes (3,). 
                First is xyz, second is rpy. 
                Represents target position for right wrist. 
                Defaults to None.
            update (bool, optional): If true, will also update visualization. Defaults to True.
        """
        if l_xyzrpy is None and r_xyzrpy is None:
            print('WARN: UPDATING ARMS WITH NO TARGET POS. NOTHINg WILL BE DONE')

        if l_xyzrpy is not None:
            self.l_target = self.l_xyz[0] + l_xyzrpy[0]
            start = time()
            l_ik = self.l_kinematics.inverse_kinematics_shoulder(
                xyz=l_xyzrpy[0],
                rpy=l_xyzrpy[1],
                current_motor_q=self.l_joints
            )
            end = time()
            print('left ik time: ', end-start)
            self.l_joints = l_ik

        if r_xyzrpy is not None:
            self.r_target = self.r_xyz[0] + r_xyzrpy[0]
            start = time()
            r_ik = self.r_kinematics.inverse_kinematics_shoulder(
                xyz = r_xyzrpy[0],
                rpy = r_xyzrpy[1],
                current_motor_q=self.r_joints
            )
            end = time()
            print('right ik time: ', end-start)
            self.r_joints = r_ik

        self.forward_kinematics(
            l_joints=self.l_joints,
            r_joints=self.r_joints,
            update=False
        )

        if update:
            self.update_arms_viz()

def main():

    viz = KinematicsVisualizer()
    joints = np.zeros(7, float)
    # joints[1] = np.pi/3
    viz.forward_kinematics(r_joints=joints, l_joints=joints)
    viz.inverse_kinematics_shoulder(
        np.array(([0.5, 0, 0], [0, 0, 0.])),
        np.array(([0.2, 0, 0], [0, 0, 0.]))
    )
    plt.pause(-1)

if __name__ == '__main__':
    main()