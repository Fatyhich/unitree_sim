import numpy as np
import argparse
from getch import getch
from testing import smooth_bringup

from decartes_controller import DecartesController
from arm_definitions import G1JointLeftArmIndex, G1JointRightArmIndex

from utils import get_rpy_component, set_rpy_component

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
        left_pose, right_pose = self.controller.get_ee_poses()
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
            'u' : self.yaw_dec


        }

    def update(self):
        self.controller.go_to(self.main_pose.homogeneous, self.off_pose.homogeneous)

    # translation actions
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

    # rotation actions
    def yaw_inc(self):
        new_val = get_rpy_component(self.main_pose, self.yaw_idx) + self.drot
        set_rpy_component(self.main_pose, self.yaw_idx, new_val)

    def yaw_dec(self):
        new_val = get_rpy_component(self.main_pose, self.yaw_idx) - self.drot
        set_rpy_component(self.main_pose, self.yaw_idx, new_val)

    def pitch_inc(self):
        new_val = get_rpy_component(self.main_pose, self.pitch_idx) + self.drot
        set_rpy_component(self.main_pose, self.pitch_idx, new_val)

    def pitch_dec(self):
        new_val = get_rpy_component(self.main_pose, self.pitch_idx) - self.drot
        set_rpy_component(self.main_pose, self.pitch_idx, new_val)

    def roll_inc(self):
        new_val = get_rpy_component(self.main_pose, self.roll_idx) + self.drot
        set_rpy_component(self.main_pose, self.roll_idx, new_val)

    def roll_dec(self):
        new_val = get_rpy_component(self.main_pose, self.roll_idx) - self.drot
        set_rpy_component(self.main_pose, self.roll_idx, new_val)

    def print_keymap(self):
        pass

    def run_teleop(self):
        print("started teleop")
        cur_cmd = '~'
        while cur_cmd != '-':
            cur_cmd = getch()
            if not cur_cmd in self.keymap.keys():
                print(f'unknown command: {cur_cmd}') 
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
    tele = KeyboardTeleop(is_local=args.local, network_interface=args.network_interface, is_left=True)
    tele.run_teleop()

if __name__ == '__main__':
    main()