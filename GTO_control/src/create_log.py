import numpy as np
import pickle

from utils.logger_visuals import LoggerVisuals

from utils.arm_definitions import G1JointIndex
real_steps = 20
max_t = 10

real_q = np.random.rand(real_steps, len(G1JointIndex))
real_dq = np.random.rand(real_steps, len(G1JointIndex))
real_ddq = np.random.rand(real_steps, len(G1JointIndex))
real_time = np.linspace(0, max_t, real_steps)

control_steps = int(real_steps/2)
target_q = np.random.rand(control_steps, len(G1JointIndex))
target_dq = np.random.rand(control_steps, len(G1JointIndex))
target_torq = np.random.rand(control_steps, len(G1JointIndex))
control_time = np.linspace(0, max_t, control_steps)

data_dict = {
    "real_q" : real_q,
    "real_dq" : real_dq,
    "real_ddq" : real_ddq,
    "real_time" : real_time,
    "target_q" : target_q,
    "target_dq" : target_dq,
    "target_torque" : target_torq,
    "control_time" : control_time
}

filename = 'test_log.pkl'

with open(filename, 'wb') as out:
    pickle.dump(data_dict, out)

logger = LoggerVisuals(local_load_file=filename)
logger.plot_full_motion(16)