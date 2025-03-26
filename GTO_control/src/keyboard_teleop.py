import numpy as np
import argparse
from getch import getch
from utils.utils import smooth_bringup

from controllers.split_decartes_controller import DecartesController
from utils.arm_definitions import G1JointLeftArmIndex, G1JointRightArmIndex

from utils.utils import get_rpy_component, set_rpy_component

DEBUG = True

class KeyboardTeleop:

    def __init__(self, is_local=False, network_interface=None, is_left=True, dpos=0.005, drot=0.01):
        self.dpos = dpos
        self.drot = drot
        self.yaw_idx = 2
        self.pitch_idx = 1
        self.roll_idx = 0

        # crate control for running that shit
        self.controller = DecartesController(
            is_in_local=is_local, 
            network_interface=network_interface
            )

        print("raising hands...")
        # raise hands slowly
        smooth_bringup(controller=self.controller)

        print("initializing poses")
        left_pose, right_pose = self.controller.get_ee_xyzrpy()
        print('left pos: ')
        print(left_pose[0])
        print('right pos:')
        print(right_pose[0])
        print('poses intialized')

        states = np.asarray(self.controller._GetJointStates())
        print('fk')
        print(self.controller.l_arm.get_ee_pose(states[:7]))

        # exit(0)

        if DEBUG:
            pass

        self.is_left = is_left
        if is_left:
            self.main_xyz = left_pose[0]
            self.main_rpy = left_pose[1]
            self.off_xyz = right_pose[0]
            self.off_rpy = right_pose[1]
        else:
            self.off_xyz = left_pose[0]
            self.off_rpy = left_pose[1]
            self.main_xyz = right_pose[0]
            self.main_rpy = right_pose[1]

        # create keymap
        self.keymap = {
            'e' : self.forward,
            'q' : self.backward,
            'a' : self.left,
            'd' : self.right,
            'w' : self.up,
            's' : self.down,
            'i' : self.roll_inc,
            'k' : self.roll_dec,
            'l' : self.pitch_inc,
            'j' : self.pitch_dec,
            'o' : self.yaw_inc,
            'u' : self.yaw_dec,
            'g' : self.get_joint_states
        }

    def update(self):
        print('left pose:')
        print('xyz: ', self.main_xyz)
        print('rpy: ', self.main_rpy)
        print('righ pose:')
        print('xyz: ', self.off_xyz)
        print('rpy: ', self.off_rpy)
        if self.is_left:
            self.controller.go_to(
                (self.main_xyz, self.main_rpy), 
                (self.off_xyz, self.off_rpy)
            )
        else:
            self.controller.go_to(
                (self.off_xyz, self.off_rpy),
                (self.main_xyz, self.main_rpy)
            )

    def get_joint_states(self):
        states = self.controller._GetJointStates()
        print('left: ')
        print(states[:7])
        print('right: ')
        print(states[7:])
        print('----------------')
        print()

    # translation actions
    def up(self):
        self.main_xyz[2] += self.dpos

    def down(self):
        self.main_xyz[2] -= self.dpos

    def left(self):
        self.main_xyz[1] += self.dpos

    def right(self):
        self.main_xyz[1] -= self.dpos

    def forward(self):
        self.main_xyz[0] += self.dpos

    def backward(self):
        self.main_xyz[0] -= self.dpos

    # rotation actions
    def yaw_inc(self):
        self.main_rpy[self.yaw_idx] += self.drot

    def yaw_dec(self):
        self.main_rpy[self.yaw_idx] -= self.drot

    def pitch_inc(self):
        self.main_rpy[self.pitch_idx] += self.drot

    def pitch_dec(self):
        self.main_rpy[self.pitch_idx] -= self.drot

    def roll_inc(self):
        self.main_rpy[self.roll_idx] += self.drot

    def roll_dec(self):
        self.main_rpy[self.roll_idx] -= self.drot

    def print_keymap(self):
        pass

    def run_teleop(self):
        print("started teleop")
        cur_cmd = '~'
        while cur_cmd != '-':
            cur_cmd = getch()
            if cur_cmd == ' ':
                smooth_bringup(self.controller)
            if not cur_cmd in self.keymap.keys():
                print(f'unknown command: {cur_cmd}') 
                continue
            self.keymap[cur_cmd]()
            if cur_cmd == 'g':
                continue
            self.update()
            

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', '--local', help="specifies whether robot is being run in a simulator", action='store_true', default=False)
    parser.add_argument('-ni', '--network-interface', help='network interface for control output', required=False)
    args = parser.parse_args()
    return args

def main():
    args = parse_args()
    tele = KeyboardTeleop(is_local=args.local, network_interface=args.network_interface, is_left=True)
    tele.run_teleop()

if __name__ == '__main__':
    main()