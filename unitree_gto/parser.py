import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt
from pathlib import Path
import re


class Parser:
    """
    A class for parsing trajectory data from CSV files and generating visualizations.
    
    This parser is designed to handle trajectory data with elbow and wrist positions/orientations,
    calculate velocities based on time deltas, and generate visualizations.
    """
    
    def __init__(self, input_file=None, output_file=None, plot_dir=None):
        """
        Initialize the Parser with file paths.
        
        Args:
            input_file (str or Path, optional): Path to the input CSV file
            output_file (str or Path, optional): Path to save the parsed data
            plot_dir (str or Path, optional): Directory to save plots
        """
        self.script_dir = Path(__file__).parent
        self.input_file = input_file or self.script_dir / "data" / "Tr.csv"
        self.output_file = output_file or self.script_dir / "data" / "parsed_trajectory.csv"
        self.plot_dir = plot_dir or self.script_dir / "data" / "plots"
        self.trajectory_data = None
    
    def parse_trajectory_file(self):
        """
        Parse the trajectory file and extract data.
        
        The file format is:
        - First line: loop = X
        - Second line: column headers
        - Remaining lines: time and position data with formatting issues
        
        Returns:
            dict: Dictionary containing parsed trajectory data and calculated velocities
        """
        with open(self.input_file, 'r') as f:
            content = f.read()
        
        # Extract all numeric values using regex
        all_values = re.findall(r'-?\d+\.?\d*', content)
        
        values = all_values[13:]  # Skip header values
        
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
                
            time = 0.02
            
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
        
        # Calculate velocities
        self._calculate_velocities(times, elbow_positions, wrist_positions, wrist_orientations)
        
        return self.trajectory_data
    
    def _calculate_velocities(self, times, elbow_positions, wrist_positions, wrist_orientations):
        """
        Calculate velocities based on positions and time deltas.
        
        Args:
            times (numpy.ndarray): Array of time points
            elbow_positions (numpy.ndarray): Array of elbow positions
            wrist_positions (numpy.ndarray): Array of wrist positions
            wrist_orientations (numpy.ndarray): Array of wrist orientations
        """
        time_deltas = 0.02
        
        elbow_velocities = np.zeros_like(elbow_positions)
        wrist_pos_velocities = np.zeros_like(wrist_positions)
        wrist_orient_velocities = np.zeros_like(wrist_orientations)
        
        # Calculate velocities for all points except the first one
        for i in range(1, len(times)):
            dt = times[i] - times[i-1]
            if dt > 0:
                elbow_velocities[i] = (elbow_positions[i] - elbow_positions[i-1]) / dt
                wrist_pos_velocities[i] = (wrist_positions[i] - wrist_positions[i-1]) / dt
                wrist_orient_velocities[i] = (wrist_orientations[i] - wrist_orientations[i-1]) / dt
        
        # Store the data in a dictionary
        self.trajectory_data = {
            'times': times,
            'elbow_positions': elbow_positions,
            'wrist_positions': wrist_positions,
            'wrist_orientations': wrist_orientations,
            'time_deltas': np.full(len(times), time_deltas),  # Fixed time delta for all points
            'elbow_velocities': elbow_velocities,
            'wrist_pos_velocities': wrist_pos_velocities,
            'wrist_orient_velocities': wrist_orient_velocities
        }
    
    def save_to_csv(self):
        """
        Save the parsed trajectory data to a CSV file.
        
        Returns:
            bool: True if successful, False otherwise
        """
        if self.trajectory_data is None:
            print("No trajectory data to save. Call parse_trajectory_file() first.")
            return False
        
        data = {
            'Time': self.trajectory_data['times'],
            'TimeDelta': self.trajectory_data['time_deltas'],
        }
        
        # Add elbow position and velocity data
        for i, axis in enumerate(['X', 'Y', 'Z']):
            data[f'Elbow{axis}'] = self.trajectory_data['elbow_positions'][:, i]
            data[f'ElbowVel{axis}'] = self.trajectory_data['elbow_velocities'][:, i]
        
        # Add wrist position and velocity data
        for i, axis in enumerate(['X', 'Y', 'Z']):
            data[f'Wrist{axis}'] = self.trajectory_data['wrist_positions'][:, i]
            data[f'WristVel{axis}'] = self.trajectory_data['wrist_pos_velocities'][:, i]
        
        # Add wrist orientation and angular velocity data
        for i, orient in enumerate(['Yaw', 'Pitch', 'Roll']):
            data[f'Wrist{orient}'] = self.trajectory_data['wrist_orientations'][:, i]
            data[f'WristVel{orient}'] = self.trajectory_data['wrist_orient_velocities'][:, i]
        
        df = pd.DataFrame(data)
        df.to_csv(self.output_file, index=False)
        print(f"Data saved to {self.output_file}")
        return True
    
    def plot_trajectory(self):
        """
        Create plots of the trajectory data.
        
        Returns:
            bool: True if successful, False otherwise
        """
        if self.trajectory_data is None:
            print("No trajectory data to plot. Call parse_trajectory_file() first.")
            return False
        
        os.makedirs(self.plot_dir, exist_ok=True)
        
        # Plot positions
        self._plot_positions()
        
        # Plot velocities
        self._plot_velocities()
        
        # 3D trajectory plot
        self._plot_3d_trajectory()
        
        print(f"Plots saved to {self.plot_dir}")
        return True
    
    def _plot_positions(self):
        """Create and save position plots for elbow and wrist."""
        plt.figure(figsize=(12, 8))
        
        # Elbow position
        plt.subplot(2, 1, 1)
        plt.plot(self.trajectory_data['times'], self.trajectory_data['elbow_positions'][:, 0], 'r-', label='Elbow X')
        plt.plot(self.trajectory_data['times'], self.trajectory_data['elbow_positions'][:, 1], 'g-', label='Elbow Y')
        plt.plot(self.trajectory_data['times'], self.trajectory_data['elbow_positions'][:, 2], 'b-', label='Elbow Z')
        plt.title('Elbow Position')
        plt.xlabel('Time (s)')
        plt.ylabel('Position')
        plt.legend()
        plt.grid(True)
        
        # Wrist position
        plt.subplot(2, 1, 2)
        plt.plot(self.trajectory_data['times'], self.trajectory_data['wrist_positions'][:, 0], 'r-', label='Wrist X')
        plt.plot(self.trajectory_data['times'], self.trajectory_data['wrist_positions'][:, 1], 'g-', label='Wrist Y')
        plt.plot(self.trajectory_data['times'], self.trajectory_data['wrist_positions'][:, 2], 'b-', label='Wrist Z')
        plt.title('Wrist Position')
        plt.xlabel('Time (s)')
        plt.ylabel('Position')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.plot_dir, 'positions.png'))
        plt.close()
    
    def _plot_velocities(self):
        """Create and save velocity plots for elbow and wrist."""
        plt.figure(figsize=(12, 12))
        
        # Elbow velocity
        plt.subplot(3, 1, 1)
        plt.plot(self.trajectory_data['times'], self.trajectory_data['elbow_velocities'][:, 0], 'r-', label='Elbow X Vel')
        plt.plot(self.trajectory_data['times'], self.trajectory_data['elbow_velocities'][:, 1], 'g-', label='Elbow Y Vel')
        plt.plot(self.trajectory_data['times'], self.trajectory_data['elbow_velocities'][:, 2], 'b-', label='Elbow Z Vel')
        plt.title('Elbow Velocity')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity')
        plt.legend()
        plt.grid(True)
        
        # Wrist position velocity
        plt.subplot(3, 1, 2)
        plt.plot(self.trajectory_data['times'], self.trajectory_data['wrist_pos_velocities'][:, 0], 'r-', label='Wrist X Vel')
        plt.plot(self.trajectory_data['times'], self.trajectory_data['wrist_pos_velocities'][:, 1], 'g-', label='Wrist Y Vel')
        plt.plot(self.trajectory_data['times'], self.trajectory_data['wrist_pos_velocities'][:, 2], 'b-', label='Wrist Z Vel')
        plt.title('Wrist Position Velocity')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity')
        plt.legend()
        plt.grid(True)
        
        # Wrist orientation velocity
        plt.subplot(3, 1, 3)
        plt.plot(self.trajectory_data['times'], self.trajectory_data['wrist_orient_velocities'][:, 0], 'r-', label='Yaw Vel')
        plt.plot(self.trajectory_data['times'], self.trajectory_data['wrist_orient_velocities'][:, 1], 'g-', label='Pitch Vel')
        plt.plot(self.trajectory_data['times'], self.trajectory_data['wrist_orient_velocities'][:, 2], 'b-', label='Roll Vel')
        plt.title('Wrist Orientation Velocity')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.plot_dir, 'velocities.png'))
        plt.close()
    
    def _plot_3d_trajectory(self):
        """Create and save 3D trajectory plot."""
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot elbow trajectory
        ax.plot(
            self.trajectory_data['elbow_positions'][:, 0],
            self.trajectory_data['elbow_positions'][:, 1],
            self.trajectory_data['elbow_positions'][:, 2],
            'r-', label='Elbow'
        )
        
        # Plot wrist trajectory
        ax.plot(
            self.trajectory_data['wrist_positions'][:, 0],
            self.trajectory_data['wrist_positions'][:, 1],
            self.trajectory_data['wrist_positions'][:, 2],
            'b-', label='Wrist'
        )
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Trajectory')
        ax.legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.plot_dir, '3d_trajectory.png'))
        plt.close()
    
    def process(self):
        """
        Process the trajectory file: parse, save to CSV, and create plots.
        
        Returns:
            dict: The trajectory data dictionary
        """
        print(f"Parsing trajectory file: {self.input_file}")
        self.parse_trajectory_file()
        
        self.save_to_csv()
        self.plot_trajectory()
        
        print("Processing complete!")
        return self.trajectory_data


def main():
    """Main function to run the parser."""
    parser = Parser()
    parser.process()


if __name__ == "__main__":
    main()