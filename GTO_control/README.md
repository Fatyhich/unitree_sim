# Logger and Logger Visuals
## General Description
These classes can be imported from `utils` module.
`JointLogger` class connects to topics where joint states and commands are being published and stores them.
By default, when `JointLogger` object is destroyed it dumps it's data to a file named `logger_dump_{time.time()}.pkl`.
`LoggerVisuals` is a child class from `JointLogger` that adds plotting functionality. Can be used as a drop-in replacement for `JointLogger`.

## Usage
### Intialization
To use the logger, just initialize it:

```python
    logger = JointLogger()
```
or
```python
    logger = LoggerVisuals()
```
And it will start logging immediately.

### Data queries
If you need to retrieve some data from the logger in it's raw form, you can use the `get_data_for_points`.
Exmaple:
```python
    data = logger.get_data_for_joints(
        joint_ids=[17, G1JointIndex.LeftWristRoll],
        desired_data=["real_q", "real_time"]
    )
```
It will return a dictionary with all requested values for all requested joints.
Note that data for each joint is in the order set in `joint_ids`.
Also, if time is queried, it will return a single array of this requested time.

The result for the query above will look something like this:
```python
    {
        "real_q" : [real_q_for_joint17, real_q_for_left_wrist_roll],
        "real_time" : whatever_the_real_time_is
    }
```

All possible keys for query:
- "real_q" - real joint states.
- "real_dq" - real joint velocities.
- "real_ddq" - real joint accelerations.
- "real_time" - times at which joints were logged.
- "target_q" - joint states that were passed as targets.
- "target_dq" - joint velocities that were passed as targets.
- "target_torque" - joint torques(tau feed forward) that were passed to control.
- "control_times" - times at which controls were recorded.

### Visualization
For now logger only supports comparing states and velocities or drawing the whole logged data.
If you want to see how left elbow joint acted, you can just use:
```python
    logger.plot_full_motion(G1JointIndex.LeftElbowJoint)
```

Or if you just want to compare velocities or states of any joint, you should use:
```python
logger.compare_joint_velocities(G1JointIndex.LeftElbowJoint)
logger.compare_joint_states(G1JointIndex.LeftElbowJoint)
```

### Loading dumps
Also logger will dump it's data when destroyed.
This can be disabled on initialization by setting `dump_on_death` parameter to False.
If you want to see what some long dead logger dumped, you can load it's file with:
```python
logger = LoggerVisuals(load_local_file=your_dumped_pickle_file)
``` 
Now it can be used for queries and plotting data.

# Visualizer
This visualizer calculates forward and backward kinematics for the G1 robot.

## Dependencies
To run, it needs some dependencies, to install them you can run:
`$conda create -f vizualizer_env.yaml`

## Usage
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

Examples can be found in `src/run_csv.py` near the end of the `main` function and in `create_log.py` which demonstrates loggers visuals.

# Trajectory Parser

This tool parses trajectory data from a CSV file containing elbow and wrist position/orientation data. It extracts target points and calculates velocities based on the time delta between recorded points.

## File Format

The expected format of the input CSV file (Tr.csv) is:
- First line: `loop = X`
- Second line: Column headers
- Remaining lines: Time and position data
- Columns are: Time, Elbow(xyz + ypr zeros), Wrist(xyzypr)

## Usage

1. Install the required dependencies:
   ```
   pip install -r requirements.txt
   ```

2. Run the parser:
   ```
   python parser.py
   ```

3. The script will:
   - Parse the trajectory data from `data/Tr.csv`
   - Calculate velocities based on time deltas
   - Save the parsed data to `data/parsed_trajectory.csv`
   - Generate visualization plots in the `data/plots` directory

### Using the Parser Class

You can also use the `Parser` class directly in your own code:

```python
from parser import Parser

# Initialize with default paths
parser = Parser()

# Or specify custom paths
parser = Parser(
    input_file="path/to/input.csv",
    output_file="path/to/output.csv",
    plot_dir="path/to/plots"
)

# Process everything at once
parser.process()

# Or step by step
parser.parse_trajectory_file()
parser.save_to_csv()
parser.plot_trajectory()

# Access the trajectory data
trajectory_data = parser.trajectory_data
```

## Output

The parser generates:
1. A CSV file with parsed trajectory data including:
   - Time and time deltas
   - Elbow positions (xyz) and velocities
   - Wrist positions (xyz) and velocities
   - Wrist orientations (ypr) and angular velocities

2. Visualization plots:
   - Position plots for elbow and wrist
   - Velocity plots for elbow and wrist
   - 3D trajectory visualization 