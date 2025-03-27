from controllers.decartes_controller import DecartesController
import numpy as np

control = DecartesController(network_interface='enp195s0f3u1')
control.go_to(np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3))

while True:
    (lp, lr), (rp, rr) = control.get_ee_xyzrpy()
    print('left: ', lp, end='\n')
    # print('right: ', rp)
