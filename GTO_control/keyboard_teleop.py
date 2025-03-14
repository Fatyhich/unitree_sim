import numpy as np
from getch import getch
from testing import smooth_bringup

from arm_ik import ArmKinematics
from synchronous_controller import SynchronousController
from arm_definitions import G1JointLeftArmIndex, G1JointRightArmIndex

class KeyboardTeleop:

    def __init__(self, is_local=False, network_interface=None, is_left=True, dpos=0.01):
        self.dpos = dpos

        # crate stuff for running that shit
        self.ik_solver = ArmKinematics()
        self.controller = SynchronousController(
            output_interface=network_interface,
            in_local=is_local
        )

        print("raising hands...")
        # raise hands slowly
        smooth_bringup(controller=self.controller)

        print("initializing poses")
        joint_states = np.array(self.controller.GetJointStates())
        left_pose, right_pose = self.ik_solver.get_forward_kinematics(joint_states)

        if is_left:
            self.main_pose = left_pose
            self.off_pose = right_pose
        else:
            self.main_pose = right_pose
            self.off_pose = left_pose

        # create keymap
        self.keymap = {
            'w' : self.forward,
            's' : self.backward,
            'a' : self.left,
            'd' : self.right,
            'c' : self.up,
            'z' : self.down

        }

    def update(self):
        new_q = self.ik_solver.solve_ik(
            l_wrist_target=self.main_pose.homogenous,
            r_wrist_target=self.off_pose.homogenous
        )

        msg = {}

        for idx, joint in enumerate(G1JointLeftArmIndex):
            msg[joint] = new_q[idx]
        
        for idx, joint in enumerate(G1JointRightArmIndex):
            msg[joint] = new_q[7 + idx]

        self.controller.ExecuteCommand(msg)


    # arm actions
    def up(self):
        self.main_pose.translation[2] += self.dpos

    def down(self):
        self.main_pose.translation[2] -= self.dpos

    def left(self):
        self.main_pose.translation[1] += self.dpos

    def right(self):
        self.main_pose.translation[1] -= self.dpos

    def forward(self):
        self.main_pose.translation[0] += self.dpos

    def backward(self):
        self.main_pose.translation[0] -= self.dpos

    def print_keymap(self):
        pass

    def run_teleop(self):
        cur_cmd = '~'
        while cur_cmd != 'q':
            cur_cmd = getch()
            self.keymap[cur_cmd]()
            

def main():
    tele = KeyboardTeleop(is_local=True)
    tele.run_teleop()

if __name__ == '__main__':
    main()