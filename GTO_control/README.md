## Visualizer
This visualizer calculates forward and backward kinematics for the G1 robot.

### Dependencies
To run, it needs some dependencies, to install them you can run:
`$conda create -f vizualizer_env.yaml`

### Usage
To visualize robot kinematics, you need to initialize the vizualizer:
```python
    viz = KinematicsVisualizer()
```

Then, just call what you want:
```python
    # basic inverse kinematics from pelvis
    viz.inverse_kinematics(
        l_xyzrpy=(l_xyz, l_rpy),
        r_xyzrpy=(r_xyz, r_rpy)
    )

    # inverse kinematics in shoulder
    viz.inverse_kinematics_shoulder(
        l_xyzrpy=(l_xyz, l_rpy),
        r_xyzrpy=(r_xyz, r_rpy)
    )
```