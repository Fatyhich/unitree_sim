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
    NON_ARM_JOINTS,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS
)

pwd = os.getcwd()

class GlobalVisualizer():
    def __init__(
            self, 
            cmd_topic='rt/arm_sdk'
    ):
        
        self.__build_models()

        # save command topic
        self.cmd_topic = cmd_topic

        # create subscriptions to all relevant topics
        self.__create_sub()

        # create visualization for robot
        self.__create_visualization()

    def __create_sub(self):
        # create sub for commands
        self.cmd_sub = ChannelSubscriber(self.command_topic, LowCmd_)
        self.cmd_sub.Init(self._add_cmd, 10)

    def __build_models(self):

        # build model from files
        self.robot = pin.RobotWrapper.BuildFromURDF(
            pwd + '/assets/g1_body29_hand14.urdf',
            pwd + '/assets/'
        )

        # reduce model to arm only
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=NON_ARM_JOINTS,
            reference_configuration=np.zeros(self.robot.model.nq, float),
        )

        # add wrist names
        self.left_wrist_name = 'left_wrist_frame'
        self.right_wrist_name = 'right_wrist_frame'

        # add elbows
        self.left_elbow_frame_name = 'left_elbow_frame'
        self.right_elbow_frame_name = 'right_elbow_frame'

        # add left wrist frame
        self.reduced_robot.model.addFrame(
            pin.Frame(self.left_wrist_name,
                      self.reduced_robot.model.getJointId('left_wrist_yaw_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.0,0,0]).T),
                      pin.FrameType.OP_FRAME)
        )
        # add right wrist frame
        self.reduced_robot.model.addFrame(
            pin.Frame(self.right_wrist_name,
                      self.reduced_robot.model.getJointId('right_wrist_yaw_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.0,0,0]).T),
                      pin.FrameType.OP_FRAME)
        )

        # add left elbow frame
        self.reduced_robot.model.addFrame(
            pin.Frame(
                self.left_elbow_frame_name,
                self.reduced_robot.model.getJointId('left_elbow_joint'),
                pin.SE3(np.eye(3),
                        np.array([0.0,0,0]).T),
                pin.FrameType.OP_FRAME
            )
        )
        # add right elbow frame
        self.reduced_robot.model.addFrame(
            pin.Frame(
                self.right_elbow_frame_name,
                self.reduced_robot.model.getJointId('right_elbow_joint'),
                pin.SE3(np.eye(3),
                        np.array([0.0,0,0]).T),
                pin.FrameType.OP_FRAME
            )
        )

        self.reduced_robot.data = pin.Data(self.reduced_robot.model)

        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()


    def __create_visualization(self):
        if self.Visualization:
            # Initialize the Meshcat visualizer for visualization
            self.vis = MeshcatVisualizer(
                self.reduced_robot.model,
                self.reduced_robot.collision_model,
                self.reduced_robot.visual_model
            )
            self.vis.initViewer(open=True) 
            self.vis.loadViewerModel("pinocchio") 
            self.vis.displayFrames(True, frame_ids=[101, 102], axis_length = 0.15, axis_width = 5)
            self.vis.display(pin.neutral(self.reduced_robot.model))

            # Enable the display of end effector target frames with short axis lengths and greater width.
            frame_viz_names = [self.ee_frame_name, self.elbow_frame_name]
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