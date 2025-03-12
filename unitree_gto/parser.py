import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt
from pathlib import Path
import re

def parse_trajectory_file(file_path):
    """
    Parse the Tr.csv file containing trajectory data.
    
    The file format is:
    - First line: loop = X
    - Second line: column headers
    - Remaining lines: time and position data with formatting issues
    
    Returns:
        dict: Dictionary containing parsed trajectory data and calculated velocities
    """
    with open(file_path, 'r') as f:
        content = f.read()
    
    # Extract all numeric values using regex
    all_values = re.findall(r'-?\d+\.?\d*', content)
    
    values = all_values[13:]
    
    values_per_row = 13
    
    num_rows = len(values) // values_per_row
    
    # Reshape the values into rows
    rows = []
    for i in range(num_rows):
        start_idx = i * values_per_row
        end_idx = start_idx + values_per_row
        row = [float(val) for val in values[start_idx:end_idx]]
        rows.append(row)
    
    # Extract data from rows
    times = []
    elbow_positions = []
    wrist_positions = []
    wrist_orientations = []
    
    for row in rows:
        if len(row) < values_per_row:
            print(f"Warning: Row has fewer than {values_per_row} values: {row}")
            continue
            
        time = row[0]
        
        # Elbow: xyz (positions 1-3), ypr are zeros (positions 4-6)
        elbow_pos = row[1:4]
        
        # Wrist: xyz (positions 7-9)
        wrist_pos = row[7:10]
        
        # Wrist: ypr (positions 10-12)
        wrist_orient = row[10:13]
        
        times.append(time)
        elbow_positions.append(elbow_pos)
        wrist_positions.append(wrist_pos)
        wrist_orientations.append(wrist_orient)
    
    times = np.array(times)
    elbow_positions = np.array(elbow_positions)
    wrist_positions = np.array(wrist_positions)
    wrist_orientations = np.array(wrist_orientations)
    
    time_deltas = 0.02
    
    # Calculate velocities
    elbow_velocities = np.zeros_like(elbow_positions)
    wrist_pos_velocities = np.zeros_like(wrist_positions)
    wrist_orient_velocities = np.zeros_like(wrist_orientations)
    
    for i in range(1, len(times)):
        dt = times[i] - times[i-1]
        if dt > 0:
            elbow_velocities[i] = (elbow_positions[i] - elbow_positions[i-1]) / dt
            wrist_pos_velocities[i] = (wrist_positions[i] - wrist_positions[i-1]) / dt
            wrist_orient_velocities[i] = (wrist_orientations[i] - wrist_orientations[i-1]) / dt
    
    trajectory_data = {
        'times': times,
        'elbow_positions': elbow_positions,
        'wrist_positions': wrist_positions,
        'wrist_orientations': wrist_orientations,
        'time_deltas': np.append(time_deltas, 0),
        'elbow_velocities': elbow_velocities,
        'wrist_pos_velocities': wrist_pos_velocities,
        'wrist_orient_velocities': wrist_orient_velocities
    }
    
    return trajectory_data

def save_to_csv(trajectory_data, output_file):
    """
    Save the parsed trajectory data to a CSV file.
    """
    data = {
        'Time': trajectory_data['times'],
        'TimeDelta': trajectory_data['time_deltas'],
    }
    
    # Add elbow position and velocity data
    for i, axis in enumerate(['X', 'Y', 'Z']):
        data[f'Elbow{axis}'] = trajectory_data['elbow_positions'][:, i]
        data[f'ElbowVel{axis}'] = trajectory_data['elbow_velocities'][:, i]
    
    # Add wrist position and velocity data
    for i, axis in enumerate(['X', 'Y', 'Z']):
        data[f'Wrist{axis}'] = trajectory_data['wrist_positions'][:, i]
        data[f'WristVel{axis}'] = trajectory_data['wrist_pos_velocities'][:, i]
    
    # Add wrist orientation and angular velocity data
    for i, orient in enumerate(['Yaw', 'Pitch', 'Roll']):
        data[f'Wrist{orient}'] = trajectory_data['wrist_orientations'][:, i]
        data[f'WristVel{orient}'] = trajectory_data['wrist_orient_velocities'][:, i]
    
    df = pd.DataFrame(data)
    df.to_csv(output_file, index=False)
    print(f"Data saved to {output_file}")

def plot_trajectory(trajectory_data, output_dir):
    """
    Create plots of the trajectory data.
    """
    os.makedirs(output_dir, exist_ok=True)
    
    plt.figure(figsize=(12, 8))
    
    plt.subplot(2, 1, 1)
    plt.plot(trajectory_data['times'], trajectory_data['elbow_positions'][:, 0], 'r-', label='Elbow X')
    plt.plot(trajectory_data['times'], trajectory_data['elbow_positions'][:, 1], 'g-', label='Elbow Y')
    plt.plot(trajectory_data['times'], trajectory_data['elbow_positions'][:, 2], 'b-', label='Elbow Z')
    plt.title('Elbow Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(trajectory_data['times'], trajectory_data['wrist_positions'][:, 0], 'r-', label='Wrist X')
    plt.plot(trajectory_data['times'], trajectory_data['wrist_positions'][:, 1], 'g-', label='Wrist Y')
    plt.plot(trajectory_data['times'], trajectory_data['wrist_positions'][:, 2], 'b-', label='Wrist Z')
    plt.title('Wrist Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'positions.png'))
    
    plt.figure(figsize=(12, 12))
    
    plt.subplot(3, 1, 1)
    plt.plot(trajectory_data['times'], trajectory_data['elbow_velocities'][:, 0], 'r-', label='Elbow X Vel')
    plt.plot(trajectory_data['times'], trajectory_data['elbow_velocities'][:, 1], 'g-', label='Elbow Y Vel')
    plt.plot(trajectory_data['times'], trajectory_data['elbow_velocities'][:, 2], 'b-', label='Elbow Z Vel')
    plt.title('Elbow Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(3, 1, 2)
    plt.plot(trajectory_data['times'], trajectory_data['wrist_pos_velocities'][:, 0], 'r-', label='Wrist X Vel')
    plt.plot(trajectory_data['times'], trajectory_data['wrist_pos_velocities'][:, 1], 'g-', label='Wrist Y Vel')
    plt.plot(trajectory_data['times'], trajectory_data['wrist_pos_velocities'][:, 2], 'b-', label='Wrist Z Vel')
    plt.title('Wrist Position Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(3, 1, 3)
    plt.plot(trajectory_data['times'], trajectory_data['wrist_orient_velocities'][:, 0], 'r-', label='Yaw Vel')
    plt.plot(trajectory_data['times'], trajectory_data['wrist_orient_velocities'][:, 1], 'g-', label='Pitch Vel')
    plt.plot(trajectory_data['times'], trajectory_data['wrist_orient_velocities'][:, 2], 'b-', label='Roll Vel')
    plt.title('Wrist Orientation Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'velocities.png'))
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(
        trajectory_data['elbow_positions'][:, 0],
        trajectory_data['elbow_positions'][:, 1],
        trajectory_data['elbow_positions'][:, 2],
        'r-', label='Elbow'
    )
    
    ax.plot(
        trajectory_data['wrist_positions'][:, 0],
        trajectory_data['wrist_positions'][:, 1],
        trajectory_data['wrist_positions'][:, 2],
        'b-', label='Wrist'
    )
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Trajectory')
    ax.legend()
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, '3d_trajectory.png'))
    
    print(f"Plots saved to {output_dir}")

def main():
    script_dir = Path(__file__).parent
    input_file = script_dir / "data" / "Tr.csv"
    output_file = script_dir / "data" / "parsed_trajectory.csv"
    plot_dir = script_dir / "data" / "plots"
    
    print(f"Parsing trajectory file: {input_file}")
    trajectory_data = parse_trajectory_file(input_file)
    
    save_to_csv(trajectory_data, output_file)
    
    plot_trajectory(trajectory_data, plot_dir)
    
    print("Processing complete!")

if __name__ == "__main__":
    main()