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