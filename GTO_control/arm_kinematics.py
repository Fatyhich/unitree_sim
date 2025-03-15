import casadi                                                                       
import meshcat.geometry as mg
import numpy as np
import pinocchio as pin                             
import time
from pinocchio import casadi as cpin                
from pinocchio.robot_wrapper import RobotWrapper    
from pinocchio.visualize import MeshcatVisualizer   
import os
import sys

from arm_definitions import NON_ARM_JOINTS

parent2_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(parent2_dir)

# from teleop.utils.weighted_moving_filter import WeightedMovingFilter


class ArmKinematics:

    def __init__(self):
        self.__build_models()
        self.__create_casadi_problem()

    def __build_models(self):

        # build model from files
        self.robot = pin.RobotWrapper.BuildFromURDF(
            '/home/oversir/humanoid_wp/GTO_control/assets/g1_body29_hand14.urdf',
            '/home/oversir/humanoid_wp/GTO_control/assets/'
        )

        # reduce model to arm only
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=NON_ARM_JOINTS,
            reference_configuration=np.array([0.0] * self.robot.model.nq),
        )

        # add left wrist frame
        self.reduced_robot.model.addFrame(
            pin.Frame('L_ee',
                      self.reduced_robot.model.getJointId('left_wrist_yaw_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.05,0,0]).T),
                      pin.FrameType.OP_FRAME)
        )
        
        # add right wrist frame
        self.reduced_robot.model.addFrame(
            pin.Frame('R_ee',
                      self.reduced_robot.model.getJointId('right_wrist_yaw_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.05,0,0]).T),
                      pin.FrameType.OP_FRAME)
        )

        self.reduced_robot.data = pin.Data(self.reduced_robot.model)

        # pin.updateFramePlacements(self.reduced_robot.model, self.reduced_robot.data)


    def __create_casadi_problem(self):
        
        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1) 
        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        # Get the hand joint ID and define the error function
        self.L_hand_id = self.reduced_robot.model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_robot.model.getFrameId("R_ee")

        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.L_hand_id].translation - self.cTf_l[:3,3],
                    self.cdata.oMf[self.R_hand_id].translation - self.cTf_r[:3,3]
                )
            ],
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    cpin.log3(self.cdata.oMf[self.L_hand_id].rotation @ self.cTf_l[:3,:3].T),
                    cpin.log3(self.cdata.oMf[self.R_hand_id].rotation @ self.cTf_r[:3,:3].T)
                )
            ],
        )

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)   # for smooth
        self.param_tf_l = self.opti.parameter(4, 4)
        self.param_tf_r = self.opti.parameter(4, 4)
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r))
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
        # self.smooth_filter = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), 14)
        self.vis = None

    def _get_forward_kinematics(self, q):

        # Compute the forward kinematics
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, q)

        # Update the frame placements
        pin.updateFramePlacements(self.reduced_robot.model, self.reduced_robot.data)

        # Get the index of the left and right end - effector frames
        left_ee_index = self.reduced_robot.model.getFrameId("L_ee", pin.FrameType.OP_FRAME)
        right_ee_index = self.reduced_robot.model.getFrameId("R_ee", pin.FrameType.OP_FRAME)

        # Retrieve the SE3 placement of the left and right end - effectors
        l_ee_placement = self.reduced_robot.data.oMf[left_ee_index]
        r_ee_placement = self.reduced_robot.data.oMf[right_ee_index]


        return l_ee_placement, r_ee_placement

    def _solve_ik(self, l_wrist_target, r_wrist_target, current_lr_arm_motor_q = None, current_lr_arm_motor_dq = None):
        if current_lr_arm_motor_q is not None:
            self.init_data = current_lr_arm_motor_q
        self.opti.set_initial(self.var_q, self.init_data)

        self.opti.set_value(self.param_tf_l, l_wrist_target)
        self.opti.set_value(self.param_tf_r, r_wrist_target)
        self.opti.set_value(self.var_q_last, self.init_data) # for smooth

        try:
            # solve optimization
            sol = self.opti.solve()
            # sol = self.opti.solve_limited()

            # get poses from solution
            sol_q = self.opti.value(self.var_q)
            # make solution smoother
            # self.smooth_filter.add_data(sol_q)
            # sol_q = self.smooth_filter.filtered_data

            # save for next zeroth approximation
            self.init_data = sol_q

            # fucking stupid lines
            # why not np.zeros_like?
            ###### UNCOMMENT IF NEEDED ########
            ## if current_lr_arm_motor_dq is not None:
            ##     v = current_lr_arm_motor_dq * 0.0
            ## else:
            ##     v = (sol_q - self.init_data) * 0.0


            # i do not trust this stuff for now
            ###### UNCOMMENT IF NEEDED ########
            ## sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))
            ##
            ## if self.Visualization:
            ##     self.vis.display(sol_q)  # for visualization

            return sol_q # , sol_tauff
        
        except Exception as e:
            print(f"ERROR in convergence, plotting debug info.{e}")

            sol_q = self.opti.debug.value(self.var_q)
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data

            if current_lr_arm_motor_dq is not None:
                v = current_lr_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q

            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            print(f"sol_q:{sol_q} \nmotorstate: \n{current_lr_arm_motor_q} \nleft_pose: \n{left_wrist} \nright_pose: \n{right_wrist}")
            if self.Visualization:
                self.vis.display(sol_q)  # for visualization

            # return sol_q, sol_tauff
            return current_lr_arm_motor_q # , np.zeros(self.reduced_robot.model.nv)


def main():
    test = ArmKinematics()
    test_q = np.zeros(14, float)
    print(test.get_forward_kinematics(test_q)[0].translation)

if __name__ == '__main__':
    main()