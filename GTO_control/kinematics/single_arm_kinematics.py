import casadi                                                                       
import meshcat.geometry as mg
import numpy as np
import pinocchio as pin                             
import time
from pinocchio import casadi as cpin
from utils.utils import SE3_from_xyz_rpy
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import MeshcatVisualizer
import os
import sys

from utils.arm_definitions import NON_ARM_JOINTS, RIGHT_ARM_JOINTS, LEFT_ARM_JOINTS

parent2_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(parent2_dir)
pwd = os.getcwd()

from utils.weighted_moving_filter import WeightedMovingFilter


class SingleArmKinematics:

    def __init__(self, is_left=True):
        self.is_left = is_left
        self.__build_models()
        self.__create_casadi_problem()

    def __build_models(self):

        # build model from files
        self.robot = pin.RobotWrapper.BuildFromURDF(
            pwd + '/assets/g1_body29_hand14.urdf',
            pwd + '/assets/'
        )

        other_arm = None
        if self.is_left:
            other_arm = RIGHT_ARM_JOINTS
        else:
            other_arm = LEFT_ARM_JOINTS

        # reduce model to arm only
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=NON_ARM_JOINTS + other_arm,
            reference_configuration=np.zeros(self.robot.model.nq, float),
        )

        # if arm is left add left frame
        self.ee_frame_name = 'EE_frame'
        self.elbow_frame_name = 'elbow_frame'
        self.shoulder_frame_name = 'shoulder_frame'
        if self.is_left:
            # add left wrist frame
            self.reduced_robot.model.addFrame(
                pin.Frame(self.ee_frame_name,
                          self.reduced_robot.model.getJointId('left_wrist_yaw_joint'),
                          pin.SE3(np.eye(3),
                                  np.array([0.0,0,0]).T),
                          pin.FrameType.OP_FRAME)
            )
            # add left elbow frame
            self.reduced_robot.model.addFrame(
                pin.Frame(
                    self.elbow_frame_name,
                    self.reduced_robot.model.getJointId('left_elbow_joint'),
                    pin.SE3(np.eye(3),
                            np.array([0.0,0,0]).T),
                    pin.FrameType.OP_FRAME
                )
            )
            # add left shoulder frame
            self.reduced_robot.model.addFrame(
                pin.Frame(
                    self.shoulder_frame_name,
                    self.reduced_robot.model.getJointId('left_shoulder_pitch_joint'),
                    pin.SE3(np.eye(3),
                            np.array([0.0,0,0]).T),
                    pin.FrameType.OP_FRAME
                )
            )
        # if arm is not left, add right frame
        else:
            # add right wrist frame
            self.reduced_robot.model.addFrame(
                pin.Frame(self.ee_frame_name,
                          self.reduced_robot.model.getJointId('right_wrist_yaw_joint'),
                          pin.SE3(np.eye(3),
                                  np.array([0.0,0,0]).T),
                          pin.FrameType.OP_FRAME)
            )
            # add right elbow frame
            self.reduced_robot.model.addFrame(
                pin.Frame(
                    self.elbow_frame_name,
                    self.reduced_robot.model.getJointId('right_elbow_joint'),
                    pin.SE3(np.eye(3),
                            np.array([0.0,0,0]).T),
                    pin.FrameType.OP_FRAME
                )
            )
            # add right shoulder frame
            self.reduced_robot.model.addFrame(
                pin.Frame(
                    self.shoulder_frame_name,
                    self.reduced_robot.model.getJointId('right_shoulder_pitch_joint'),
                    pin.SE3(np.eye(3),
                            np.array([0.0,0,0]).T),
                    pin.FrameType.OP_FRAME
                )
            )

        self.reduced_robot.data = pin.Data(self.reduced_robot.model)

        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()



    def __create_casadi_problem(self):
        
        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1) 
        self.target_tf = casadi.SX.sym("target_tf", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        # Get the hand joint ID and define the error function
        self.EE_id = self.reduced_robot.model.getFrameId(self.ee_frame_name)

        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.target_tf],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.EE_id].translation - self.target_tf[:3,3]
                )
            ],
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.target_tf],
            [
                casadi.vertcat(
                    cpin.log3(self.cdata.oMf[self.EE_id].rotation @ self.target_tf[:3,:3].T)
                )
            ],
        )

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)   # for smooth
        self.param_tf = self.opti.parameter(4, 4)
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf))
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf))
        self.regularization_cost = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        # Setting optimization constraints and goals
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit)
        )
        self.opti.minimize(50 * self.translational_cost + self.rotation_cost + 0.02 * self.regularization_cost + 0.1 * self.smooth_cost)

        opts = {
            'ipopt':{
                'print_level':0,
                'max_iter':50,
                'tol':1e-6
            },
            'print_time':False,# print or not
            'calc_lam_p':False # https://github.com/casadi/casadi/wiki/FAQ:-Why-am-I-getting-%22NaN-detected%22in-my-optimization%3F
        }
        self.opti.solver("ipopt", opts)

        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.smooth_filter = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), 7)
        self.vis = None

    def _update_forward_kinematics(self, q):
        # Compute the forward kinematics
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, q)

        # Update the frame placements
        pin.updateFramePlacements(self.reduced_robot.model, self.reduced_robot.data)

    def get_ee_pose(self, q):
        self._update_forward_kinematics(q)

        # Get the index of the left and right end - effector frames
        ee_index = self.reduced_robot.model.getFrameId(self.ee_frame_name, pin.FrameType.OP_FRAME)

        # Retrieve the SE3 placement of the left and right end - effectors
        ee_placement = self.reduced_robot.data.oMf[ee_index]

        return ee_placement

    def get_shoulder_pose(self, q):
        self._update_forward_kinematics(q)
        # get shoulder index
        shoulder_index = self.reduced_robot.model.getFrameId(self.shoulder_frame_name, pin.FrameType.OP_FRAME)
        # get shoulder placement
        shoulder_pose = self.reduced_robot.data.oMf[shoulder_index]
        return shoulder_pose

    def get_arm_points(self, q):
        # WRIST
        ee_pose = self.get_ee_pose(q)

        # ELBOW
        # get elbow index
        elbow_index = self.reduced_robot.model.getFrameId(self.elbow_frame_name, pin.FrameType.OP_FRAME)
        # get elbow SE3 placement
        elbow_pose = self.reduced_robot.data.oMf[elbow_index]

        # SHOULDER
        shoulder_pose = self.get_shoulder_pose(q)
        
        return shoulder_pose, elbow_pose, ee_pose

    def pelvis_to_shoulder(self, pose:pin.SE3):
        # get shoulder
        shoulder_pose:pin.SE3 = self.get_shoulder_pose(np.zeros(7))

        # apply inverse transform
        target_pose_shoulder = shoulder_pose.act(pose)

        return target_pose_shoulder

    def pelvis_to_shoulder_xyz(self, xyz):
        shoulder_pose = self.get_shoulder_pose(np.zeros(7))
        result = shoulder_pose.rotation @ xyz
        result = result + shoulder_pose.translation
        return result

    def inverse_kinematics(self, xyz, rpy, current_motor_q=None, current_motor_dq=None, from_shoulder:bool=False):
        wrist_target = SE3_from_xyz_rpy(xyz, rpy)

        if from_shoulder:
            wrist_target = self.pelvis_to_shoulder(wrist_target)

        return self._solve_ik(
            wrist_target=wrist_target.homogeneous,
            current_arm_motor_q=current_motor_q,
            current_arm_motor_dq=current_motor_dq
            )

    def _solve_ik(self, wrist_target, current_arm_motor_q = None, current_arm_motor_dq = None):
        if current_arm_motor_q is not None:
            self.init_data = current_arm_motor_q
        self.opti.set_initial(self.var_q, self.init_data)

        self.opti.set_value(self.param_tf, wrist_target)
        self.opti.set_value(self.var_q_last, self.init_data) # for smooth

        try:
            # solve optimization
            sol = self.opti.solve()
            # sol = self.opti.solve_limited()

            # get poses from solution
            sol_q = self.opti.value(self.var_q)
            # make solution smoother
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data

            # save for next zeroth approximation
            self.init_data = sol_q

            # get target velocities
            if current_arm_motor_dq is not None:
                v = current_arm_motor_dq
            else:
                v = sol_q - self.init_data


            # i do not trust this stuff for now
            ###### UNCOMMENT IF NEEDED ########
            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))
            ##
            ## if self.Visualization:
            ##     self.vis.display(sol_q)  # for visualization

            return sol_q, sol_tauff
        
        except Exception as e:
            print(f"ERROR in convergence, plotting debug info.{e}")

            sol_q = self.opti.debug.value(self.var_q)
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data

            if current_arm_motor_dq is not None:
                v = current_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q

            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            print(f"sol_q:{sol_q} \nmotorstate: \n{current_arm_motor_q} \npose: \n{wrist_target}")
            if self.Visualization:
                self.vis.display(sol_q)  # for visualization

            # return sol_q, sol_tauff
            return current_arm_motor_q # , np.zeros(self.reduced_robot.model.nv)


def main():
    test = SingleArmKinematics()
    test_q = np.zeros(14, float)
    print(test.get_forward_kinematics(test_q)[0].translation)

if __name__ == '__main__':
    main()