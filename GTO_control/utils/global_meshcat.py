import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
from pinocchio import casadi as cpin
import numpy as np
import meshcat.geometry as mg
import os

from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

# maybe CRC is not needed
from unitree_sdk2py.utils.crc import CRC

from utils.arm_definitions import (
    ALL_JOINTS,
    NON_ARM_JOINTS,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    FINGER_JOINTS,
    G1JointIndex
)

pwd = os.getcwd()

class GlobalVisualizer():

    def __init__(
            self, 
            cmd_topic='rt/arm_sdk'
    ):
        
        self.__build_models()

        # create visualization for robot
        self.__create_visualization()

        # save command topic
        self.command_topic = cmd_topic

        # create subscriptions to all relevant topics
        self.__create_sub()

    def __cmd_callback(self, msg:LowCmd_):
        q = np.zeros(29)
        # print(len(q))
        for idx in range(29):
            q[idx] = msg.motor_cmd[idx].q

        self.visualize(q)

    def visualize(self, q):
        self.vis.display(q)

    def _update_forward_kinematics(self, q):
        # Compute the forward kinematics
        pin.forwardKinematics(self.robot.model, self.robot.data, q)

        # Update the frame placements
        pin.updateFramePlacements(self.robot.model, self.robot.data)

    def __create_sub(self):
        # create sub for commands
        self.cmd_sub = ChannelSubscriber(self.command_topic, LowCmd_)
        self.cmd_sub.Init(self.__cmd_callback, 10)

    def __build_models(self):

        # build model from files
        self.robot = pin.RobotWrapper.BuildFromURDF(
            pwd + '/assets/g1_body29_hand14.urdf',
            pwd + '/assets/'
        )

        self.robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=FINGER_JOINTS,
            reference_configuration=np.zeros(self.robot.model.nq, float)
        )

        # for joint_id in range(self.robot.model.njoints):
        #     print(self.robot.model.names[joint_id])

        # add wrist names
        self.left_wrist_name = 'left_wrist_frame'
        self.right_wrist_name = 'right_wrist_frame'

        # add elbows
        self.left_elbow_frame_name = 'left_elbow_frame'
        self.right_elbow_frame_name = 'right_elbow_frame'

        # add left wrist frame
        self.robot.model.addFrame(
            pin.Frame(self.left_wrist_name,
                      self.robot.model.getJointId('left_wrist_yaw_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.0,0,0]).T),
                      pin.FrameType.OP_FRAME)
        )
        # add right wrist frame
        self.robot.model.addFrame(
            pin.Frame(self.right_wrist_name,
                      self.robot.model.getJointId('right_wrist_yaw_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.0,0,0]).T),
                      pin.FrameType.OP_FRAME)
        )

        # add left elbow frame
        self.robot.model.addFrame(
            pin.Frame(
                self.left_elbow_frame_name,
                self.robot.model.getJointId('left_elbow_joint'),
                pin.SE3(np.eye(3),
                        np.array([0.0,0,0]).T),
                pin.FrameType.OP_FRAME
            )
        )
        # add right elbow frame
        self.robot.model.addFrame(
            pin.Frame(
                self.right_elbow_frame_name,
                self.robot.model.getJointId('right_elbow_joint'),
                pin.SE3(np.eye(3),
                        np.array([0.0,0,0]).T),
                pin.FrameType.OP_FRAME
            )
        )

        self.robot.data = pin.Data(self.robot.model)

        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.robot.model)
        self.cdata = self.cmodel.createData()

    def __create_visualization(self):
        # Initialize the Meshcat visualizer for visualization
        self.vis = MeshcatVisualizer(
            self.robot.model,
            self.robot.collision_model,
            self.robot.visual_model
        )
        self.vis.initViewer(open=True) 
        self.vis.loadViewerModel("pinocchio") 
        self.vis.displayFrames(True, frame_ids=[101, 102], axis_length = 0.15, axis_width = 5)
        self.vis.display(pin.neutral(self.robot.model))

        # Enable the display of end effector target frames with short axis lengths and greater width.
        frame_viz_names = [
            self.left_elbow_frame_name, self.left_wrist_name, 
            self.right_elbow_frame_name, self.right_wrist_name
            ]
        FRAME_AXIS_POSITIONS = (
            np.array([[0, 0, 0], [1, 0, 0],
                      [0, 0, 0], [0, 1, 0],
                      [0, 0, 0], [0, 0, 1]]).astype(np.float32).T
        )
        FRAME_AXIS_COLORS = (
            np.array([[1, 0, 0], [1, 0.6, 0],
                      [0, 1, 0], [0.6, 1, 0],
                      [0, 0, 1], [0, 0.6, 1]]).astype(np.float32).T
        )
        axis_length = 0.1
        axis_width = 10
        for frame_viz_name in frame_viz_names:
            self.vis.viewer[frame_viz_name].set_object(
                mg.LineSegments(
                    mg.PointsGeometry(
                        position=axis_length * FRAME_AXIS_POSITIONS,
                        color=FRAME_AXIS_COLORS,
                    ),
                    mg.LineBasicMaterial(
                        linewidth=axis_width,
                        vertexColors=True,
                    ),
                )
            )
