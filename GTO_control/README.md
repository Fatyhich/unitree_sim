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