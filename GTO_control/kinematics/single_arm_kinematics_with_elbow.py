import casadi
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin
from utils.utils import SE3_from_xyz_rpy 
from kinematics.single_arm_kinematics import SingleArmKinematics

class SingleArmKinematicsWithElbow(SingleArmKinematics):

    def __init__(self, is_left=True):
        SingleArmKinematics.__init__(self, is_left)
        self.__create_casadi_problem_with_elbow()

    def __create_casadi_problem_with_elbow(self):
        
        # Creating symbolic variables
        # self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1) 
        # self.target_tf = casadi.SX.sym("target_tf", 4, 4)
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

        self.elbow_id = self.reduced_robot.model.getFrameId(self.elbow_frame_name)
        self.elbow_target_translation = casadi.SX.sym("elbow_target_translation", 3)

        self.elbow_translational_error = casadi.Function(
            "elbow_translational_error",
            [self.cq, self.elbow_target_translation],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.elbow_id].translation - self.elbow_target_translation
                )
            ]
        )

        # create optimization variables
        self.elbow_opti = casadi.Opti()
        self.elbow_var_q          = self.elbow_opti.variable(self.reduced_robot.model.nq)
        self.elbow_var_q_last     = self.elbow_opti.parameter(self.reduced_robot.model.nq)   # for smooth
        self.wrist_param_tf       = self.elbow_opti.parameter(4, 4)
        self.elbow_param_pos      = self.elbow_opti.parameter(3)
        # create costs
        self.wrist_translational_cost  = casadi.sumsqr(self.translational_error(self.elbow_var_q, self.wrist_param_tf))
        self.wrist_rotation_cost       = casadi.sumsqr(self.rotational_error(self.elbow_var_q, self.wrist_param_tf))
        self.elbow_regularization_cost       = casadi.sumsqr(self.elbow_var_q)
        self.elbow_smooth_cost               = casadi.sumsqr(self.elbow_var_q - self.elbow_var_q_last)
        self.elbow_translational_cost        = casadi.sumsqr(self.elbow_translational_error(self.elbow_var_q, self.elbow_param_pos))

        self.joint_weigths = [0, 0, 0, 0, 0, 1, 0]
        self.joint_cost = casadi.sumsqr(self.elbow_var_q * self.joint_weigths)

        # Setting optimization constraints and goals
        self.elbow_opti.subject_to(self.elbow_opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.elbow_var_q,
            self.reduced_robot.model.upperPositionLimit)
        )
        self.elbow_opti.minimize(
            50 * self.wrist_translational_cost + 
            self.wrist_rotation_cost + 
            0.02 * self.elbow_regularization_cost + 
            0.1 * self.elbow_smooth_cost +
            50 * self.elbow_translational_cost +
            0 * self.joint_cost
        )
        print('am i even here?')

        opts = {
            'ipopt':{
                'print_level':0,
                'max_iter':50,
                'tol':1e-6
            },
            'print_time':False,# print or not
            'calc_lam_p':False # https://github.com/casadi/casadi/wiki/FAQ:-Why-am-I-getting-%22NaN-detected%22in-my-optimization%3F
        }
        self.elbow_opti.solver("ipopt", opts)

        self.init_data = np.zeros(self.reduced_robot.model.nq)
        # self.smooth_filter = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), 14)

    def __solve_ik_elbow(
            self,
            wrist_target, 
            elbow_target_xyz,
            current_arm_motor_q=None, 
            current_arm_motor_dq=None,
            ):
        # set init guess
        if current_arm_motor_dq is not None:
            self.init_data = current_arm_motor_q
        self.elbow_opti.set_initial(
            self.elbow_var_q,
            self.init_data
        )

        self.elbow_opti.set_value(self.wrist_param_tf, wrist_target)
        self.elbow_opti.set_value(self.elbow_param_pos, elbow_target_xyz)
        self.elbow_opti.set_value(self.elbow_var_q_last, self.init_data)

        try:
            # optimize
            sol = self.elbow_opti.solve()

            # get q joints
            sol_q = self.elbow_opti.value(self.elbow_var_q)

            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data

            # save for next zeroth approximation
            self.init_data = sol_q

            # get target velocities
            if current_arm_motor_dq is not None:
                v = current_arm_motor_dq
            else:
                v = sol_q - self.init_data

            
            if self.Visualization:
                self.vis.display(sol_q)  # for visualization

            # i do not trust this stuff for now
            ###### UNCOMMENT IF NEEDED ########
            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            return sol_q, sol_tauff

        except Exception as e:
            print(f"ERROR in convergence, plotting debug info.{e}")

            sol_q = self.elbow_opti.debug.value(self.elbow_var_q)
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data

            if current_arm_motor_dq is not None:
                v = current_arm_motor_dq#  * 0.0
            else:
                v = (sol_q - self.init_data)# * 0.0

            self.init_data = sol_q

            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            print(f"sol_q:{sol_q} \nmotorstate: \n{current_arm_motor_q} \npose: \n{wrist_target}")
            if self.Visualization:
                self.vis.display(sol_q)  # for visualization

            # return sol_q, sol_tauff
            return current_arm_motor_q, np.zeros(self.reduced_robot.model.nv)
        
    def inverse_kinematics(
            self, 
            xyz, 
            rpy, 
            elbow_xyz=None, 
            current_motor_q=None, 
            current_motor_dq=None,
            from_shoulder:bool=False
        ):
        # if no elbow given, return just regular kinematics
        if elbow_xyz is None:
            return super().inverse_kinematics(xyz, rpy, current_motor_q, current_motor_dq, from_shoulder=from_shoulder)
        
        # otherwise calculate ik for elbow
        wrist_target = SE3_from_xyz_rpy(xyz, rpy)
        if from_shoulder:
            wrist_target = self.pelvis_to_shoulder(wrist_target)
            elbow_xyz = self.pelvis_to_shoulder_xyz(elbow_xyz)
        return self.__solve_ik_elbow(
            wrist_target=wrist_target.homogeneous,
            elbow_target_xyz=elbow_xyz,
            current_arm_motor_q=current_motor_q,
            current_arm_motor_dq=current_motor_dq
            )
    