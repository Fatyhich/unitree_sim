Hello from Unitree sim
![](./doc/unitree_h1_sim.png)

# Related links
- [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)
- [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
- [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)
- [Unitree Doc](https://support.unitree.com/home/zh/developer)
- [Mujoco Doc](https://mujoco.readthedocs.io/en/stable/overview.html)

# Installation
## 0. Clone this repo
Please clone this repo to your machine:
```bash
git clone git@github.com:Fatyhich/unitree_sim.git
```

## 1. System Dependencies And Environment
Source `ubuntu_install_deps.sh` to install environment.
```bash
cd unitree_sim
source ubuntu_install_deps.sh
```
This script will install micromamba if it is not installed and create environment named `unitree_sim_env` that contains all the dependencies for running simulator and developing for G1.
**NOTE: For now only Zsh and Bash are supported.**
This command will install all system requirements for running the simulator and activate the corresponding python environment.

```

## 2. Starting Simulator
```bash
sudo apt-get update & upgrade -y
cd unitree_mujoco/simulate_python 
python ./unitree_mujoco.py
```
As a marker of success, you will see a new window with the simulator and robot appear! 
