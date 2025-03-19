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
## 1. System Dependencies
Please clone this repo to your machine:
```bash
git clone git@github.com:Fatyhich/unitree_sim.git
```

Depending on your shell, execute `ubuntu_install_deps.sh`.
```bash
cd unitree_sim
zsh ubuntu_install_deps.sh
```
This command will install all system requirements for running the simulator.

## 2. Python Dependencies
To install all the python dependencies, just run:
```bash
cd unitree_sdk2_python
conda create -f python_deps.yaml
python -m pip install -e .
cd ..
```

## 2. Starting Simulator
```bash
sudo apt-get update & upgrade -y
cd unitree_mujoco/simulate_python 
python3 ./unitree_mujoco.py
```
As a marker of success, you will see a new window with the simulator and robot appear! 
