import numpy as np
import argparse
from getch import getch
from testing import smooth_bringup

from arm_kinematics import ArmKinematics
from synchronous_controller import SynchronousController
from arm_definitions import G1JointLeftArmIndex, G1JointRightArmIndex

class KeyboardTeleop:

    def __init__(self, is_local=False, network_interface=None, is_left=True, dpos=0.005):
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
        print('poses intialized')

        if is_left:
            self.main_pose = left_pose
            self.off_pose = right_pose
        else:
            self.main_pose = right_pose
            self.off_pose = left_pose

        self.update()

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
            l_wrist_target=self.main_pose.homogeneous,
            r_wrist_target=self.off_pose.homogeneous
        )
        msg = {}

        for idx, joint in enumerate(G1JointLeftArmIndex):
            msg[joint] = (new_q[idx], 0)
        
        for idx, joint in enumerate(G1JointRightArmIndex):
            msg[joint] = (new_q[7 + idx], 0)

        self.controller.ExecuteCommand(msg)


    # arm actions
    def up(self):
        print("up")
        self.main_pose.translation[2] += self.dpos

    def down(self):
        print("down")
        self.main_pose.translation[2] -= self.dpos

    def left(self):
        print("left")
        self.main_pose.translation[1] += self.dpos

    def right(self):
        print("right")
        self.main_pose.translation[1] -= self.dpos

    def forward(self):
        print("fwd")
        self.main_pose.translation[0] += self.dpos

    def backward(self):
        self.main_pose.translation[0] -= self.dpos

    def print_keymap(self):
        pass

    def run_teleop(self):
        print("started teleop")
        cur_cmd = '~'
        while cur_cmd != 'q':
            cur_cmd = getch()
            self.keymap[cur_cmd]()
            self.update()
            

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', '--local', help="specifies whether robot is being run in a simulator", action='store_true')
    parser.add_argument('-ni', '--network-interface', help='network interface for control output', required=False)
    args = parser.parse_args()
    return args

def main():
    args = parse_args()
    tele = KeyboardTeleop(is_local=args.local, network_interface=args.network_interface)
    tele.run_teleop()

if __name__ == '__main__':
    main()