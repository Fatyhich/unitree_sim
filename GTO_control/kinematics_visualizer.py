import matplotlib.pyplot as plt
import numpy as np
from single_arm_kinematics import SingleArmKinematics
from csv_parser import Parser

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
        # fill arrays with zero poses
        # and draw them
        self.set_arm_joints(np.zeros(7, float), np.zeros(7, float))
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

    def update_arms_viz(self):
        # clear everything
        self.ax.cla()
        # draw arms
        self.draw_arm(self.l_xyz, color=[0, 0, 1])
        self.draw_arm(self.r_xyz, color=[1, 0, 0])
        # set axes
        self.set_axes()
        plt.pause(0.01)

    def get_arm_points_fk(self, joints, left:bool):
        if left:
            shoulder_pos, elbow_pos, ee_pos = self.l_kinematics.get_arm_points(joints)
        else:
            shoulder_pos, elbow_pos, ee_pos = self.r_kinematics.get_arm_points(joints)

        return np.array([
            shoulder_pos.translation,
            elbow_pos.translation, 
            ee_pos.translation
        ])

    def set_arm_joints(self, l_joints=None, r_joints=None):
        if l_joints is None and r_joints is None:
            print('WARN: UPDATING ARMS WITH NO JOINT POS. NOTHIN WILL BE DONE')
            return
        
        if l_joints is not None:
            self.l_xyz = self.get_arm_points_fk(l_joints, left=True)
        
        if r_joints is not None:
            self.r_xyz = self.get_arm_points_fk(r_joints, left=False)

        self.update_arms_viz()
        

    # def visualize_fk(self, l_joints=np.zeros(7, float), r_joints=np.zeros(7, float)):


def main():

    viz = KinematicsVisualizer()
    joints = np.zeros(7, float)
    # joints[1] = np.pi/3
    viz.set_arm_joints(r_joints=joints, l_joints=joints)
    plt.pause(-1)

if __name__ == '__main__':
    main()