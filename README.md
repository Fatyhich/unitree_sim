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
## 1. Dependencies
Please clone this repo to your machine:
```bash
git clone git@github.com:Fatyhich/unitree_sim.git
```

Repo contains Dockerfile with all-you-need dependencies, so to work with project you need to build docker image:
```bash
cd unitree_sim
docker build -t unitree_sim .
```
This command build image with name `unitree_sim`, now we can run container from this image:
```bash
xhost +
docker run -it --name unitree_h1 \
        --privileged -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /PATH/TO/FOLDER/WITH/REPO:/home/oversir/humanoid_sim unitree_sim
```
**Important Note** You need to change last line according to your case, for example, if you clone repo to home directory path to folder with repo might looks like `/home/USER/unitree_sim`

## 2. Starting Simulator
To run simulator with our robot we need:
```bash
sudo apt-get update & upgrade -y
cd humanoid_sim/unitree_sdk2_python
pip install -e .
```
This will install `unitree_sdk2_python` to our system in container. On next step we exactly run simulator:
```bash
cd
cd humanoid_sim/unitree_sdk2_python
python3 ./unitree_mujoco.py
```
As a marker of success we receive new window with simulator and robot appeared ! 