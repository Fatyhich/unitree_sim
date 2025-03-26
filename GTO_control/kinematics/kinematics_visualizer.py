import matplotlib.pyplot as plt
from time import time
import numpy as np
import pinocchio as pin
from kinematics.single_arm_kinematics_with_elbow import SingleArmKinematicsWithElbow
# from csv_parser import Parser

class KinematicsVisualizer():

    def __init__(self):
        self.l_kinematics = SingleArmKinematicsWithElbow(is_left=True)
        self.r_kinematics = SingleArmKinematicsWithElbow(is_left=False)
        # add plot to draw on
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        # set axis parameters
        self.MAX_DIM = 0.4
        self.X_CENTER = 0
        self.Y_CENTER = 0
        self.Z_CENTER = 0

        self.pause = 0.01

        # create arrays for arm poses
        self.l_xyz = None
        self.r_xyz = None
        self.l_rpy = None
        self.r_rpy = None
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

    def draw_axes(self, point, rpy, axes_len=0.05, axes_thickness=2, alternate_color=False):
        red = [1, 0, 0]
        green = [0, 1, 0]
        blue = [0, 0, 1]
        if alternate_color:
            red = [1, 0, 0.5]
            green = [0.5, 1, 0.2]
            blue = [0, 1, 1]

        # create rotation matirx
        rotation_matrix = pin.rpy.rpyToMatrix(rpy) * axes_len
        
        x_axis = np.array([point, rotation_matrix[0] + point])
        y_axis = np.array([point, rotation_matrix[1] + point])
        z_axis = np.array([point, rotation_matrix[2] + point])

        self.ax.plot(
            xs=x_axis[:, 0],
            ys=x_axis[:, 1],
            zs=x_axis[:, 2],
            color=red,
            linewidth=axes_thickness
        )
        self.ax.plot(
            xs=y_axis[:, 0],
            ys=y_axis[:, 1],
            zs=y_axis[:, 2],
            color=green,
            linewidth=axes_thickness
        )
        self.ax.plot(
            xs=z_axis[:, 0], 
            ys=z_axis[:, 1], 
            zs=z_axis[:, 2], 
            color=blue,
            linewidth=axes_thickness
        )

    def draw_arm(self, points_xyz, points_rpy=None, color=[0., 0., 1.], marker='o'):
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

        if points_rpy is not None:
            for xyz, rpy in zip(points_xyz, points_rpy):
                self.draw_axes(xyz, rpy)


    def draw_target(self, xyzrpy, color=[0, 0, 1], marker='x'):
        if xyzrpy is None:
            return
        
        xyz = xyzrpy[0]
        rpy = xyzrpy[1]

        self.ax.scatter(
            xs=xyz[0],
            ys=xyz[1],
            zs=xyz[2],
            color=color,
            marker=marker,
            s=70
        )

        if rpy is not None:
            self.draw_axes(xyz, rpy, alternate_color=True)

    def update_arms_viz(self):
        # clear everything
        self.ax.cla()
        # draw arms
        self.draw_arm(self.l_xyz, points_rpy=self.l_rpy, color=[0, 0, 1])
        self.draw_arm(self.r_xyz, points_rpy=self.r_rpy, color=[1, 0, 0])
        # draw targets
        self.draw_target(self.l_target, color=[0, 0, 1])
        self.draw_target(self.r_target, color=[1, 0, 0])
        # set axes
        self.set_axes()
        plt.pause(self.pause)

    def __get_arm_points_fk(self, left:bool):
        if left:
            shoulder_pos, elbow_pos, ee_pos = self.l_kinematics.get_arm_points(self.l_joints)
        else:
            shoulder_pos, elbow_pos, ee_pos = self.r_kinematics.get_arm_points(self.r_joints)

        return np.array([
            shoulder_pos.translation,
            elbow_pos.translation, 
            ee_pos.translation
        ]), np.array([
            pin.rpy.matrixToRpy(shoulder_pos.rotation),
            pin.rpy.matrixToRpy(elbow_pos.rotation),
            pin.rpy.matrixToRpy(ee_pos.rotation)
        ])

    def forward_kinematics(self, l_joints=None, r_joints=None, update=True):
        if l_joints is None and r_joints is None:
            print('WARN: UPDATING ARMS WITH NO JOINT POS. NOTHIN WILL BE DONE')
            return
        
        if l_joints is not None:
            self.l_joints = l_joints
            self.l_xyz, self.l_rpy = self.__get_arm_points_fk(left=True)
        
        if r_joints is not None:
            self.r_joints = r_joints
            self.r_xyz, self.r_rpy = self.__get_arm_points_fk(left=False)
        if update:
            self.update_arms_viz()
        
    def pelvis_to_shoulder(self, l_xyzrpy=None, r_xyzrpy=None):
        pass

    
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
            self.l_target = l_xyzrpy
            l_ik = self.l_kinematics.inverse_kinematics(
                xyz=l_xyzrpy[0],
                rpy=l_xyzrpy[1],
                current_motor_q=self.l_joints
            )
            self.l_joints = l_ik

        if r_xyzrpy is not None:
            self.r_target = r_xyzrpy
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

    def inverse_kinematics_shoulder(
            self, 
            l_xyzrpy:tuple=None, 
            r_xyzrpy:tuple=None, 
            update=True, 
            l_elbow_xyz=None,
            r_elbow_xyz=None
            ):
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
            self.l_target = np.array(l_xyzrpy)
            self.l_target[0] += self.l_xyz[0]
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
            self.r_target = np.array(r_xyzrpy)
            self.r_target[0] += self.r_xyz[0]
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


# NEVERMIND, ILL FINISH THAT LATER

    def inverse_kinematics(
            self,
            l_xyzrpy:tuple=None,
            r_xyzrpy:tuple=None,
            origin:str='pelvis',
            l_elbow_target=None,
            r_elbow_target=None
            ):
        """Calculates and draws inverse kinematics for given targets.

        Args:
            l_xyzrpy (tuple, optional): Target position and orientation of the left wrist **as a tuple** (xyz, rpy). 
                If **None**, will use last known position of the left wrist.
                Defaults to None.

            r_xyzrpy (tuple, optional): Target position and orientation of the right wrist **as a tuple** (xyz, rpy). 
                If **None**, will use last known position of the right wrist.
                Defaults to None.

            origin (str, optional): Describes where the origin of coordinates is. 
                It can either be 'pelvis' or 'shoulder'.
                Defaults to 'pelvis'.

            l_elbow_target (_type_, optional): Target xyz of the left elbow. 
                If **None** will not use elbow for inverse kinematics.
                Defaults to None.
            r_elbow_target (_type_, optional): Target xyz of the right elbow. 
                If **None** will not use elbow for inverse kinematics.
                Defaults to None.
        """

        # separate targets
        if l_xyzrpy is not None:
            self.l_target = np.array(l_xyzrpy)
            l_xyz, l_rpy = l_xyzrpy

        if r_xyzrpy is not None:
            self.r_target = np.array(r_xyzrpy)
            r_xyz, r_rpy = r_xyzrpy
        

        # if origin is shoulder, convert to shoulder origin
        if origin == 'shoulder':
            if l_xyzrpy is not None:
                self.l_target[0] += self.l_xyz[0]
            if r_xyzrpy is not None:
                self.r_target[0] += self.r_xyz[0]

        # solve kinematics
        if l_xyzrpy is not None:
            start = time()
            l_ik = self.l_kinematics.inverse_kinematics(
                xyz=l_xyz,
                rpy=l_rpy,
                elbow_xyz=l_elbow_target,
                current_motor_q=self.l_joints
            )
            end = time()
            print(f'left kinematics done in {end-start} sec')
            self.l_joints = l_ik

        if r_xyzrpy is not None:
            start = time()
            r_ik = self.r_kinematics.inverse_kinematics(
                xyz=r_xyz,
                rpy=r_rpy,
                elbow_xyz=r_elbow_target,
                current_motor_q=self.r_joints
            )
            end = time()
            print(f'right kinematics done in {end-start} sec')
            self.r_joints = r_ik


        self.forward_kinematics(
            l_joints=self.l_joints,
            r_joints=self.r_joints,
            update=False
        )

        self.update_arms_viz()
               

def main():

    viz = KinematicsVisualizer()
    joints = np.zeros(7, float)
    # joints[1] = np.pi/3
    viz.forward_kinematics(r_joints=joints, l_joints=joints)
    viz.inverse_kinematics_shoulder(
        np.array(([0.5, 0, 0], [0, 0, 0.])),
        np.array(([0.3, -0.2, 0], [0, 0, np.pi/2.]))
    )
    plt.pause(-1)

if __name__ == '__main__':
    main()