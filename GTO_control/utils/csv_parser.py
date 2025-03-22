import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt
from pathlib import Path
import re
from scipy.spatial.transform import Rotation


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
        self.transform_matrix = None
    
    def parse_trajectory_file(self):
        """
        Parse the trajectory file and extract data.
        
        Args:
            transform (bool): Whether to transform coordinates using the transform matrix
            inverse (bool): If True, performs inverse transformation
            
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
        self.rows = []
        for i in range(num_rows):
            start_idx = i * values_per_row
            end_idx = start_idx + values_per_row
            row = [float(val) for val in values[start_idx:end_idx]]
            self.rows.append(row)
        
        # Extract data from rows
        times = []
        elbow_positions = []
        wrist_positions = []
        wrist_orientations = []
        
        for row in self.rows:
            if len(row) < values_per_row:
                print(f"Warning: Row has fewer than {values_per_row} values: {row}")
                continue
                
            time = row[0]
            
            # Elbow: xyz (positions 1-3), ypr are zeros (positions 4-6)
            elbow_pos = row[1:4]
            elbow_pos[0], elbow_pos[1] = elbow_pos[1], elbow_pos[0]
            elbow_pos = [x * 0.3 for x in elbow_pos]
            
            # Wrist: xyz (positions 7-9)
            wrist_pos = row[7:10]
            wrist_pos[0], wrist_pos[1] = wrist_pos[1], wrist_pos[0]
            wrist_pos = [x * 0.4 for x in wrist_pos]
            
            # Wrist: ypr (positions 10-12)
            wrist_orient = row[10:13]
            wrist_orient.reverse()
            wrist_orient[0] = wrist_orient[0] + 180
            wrist_orient[2] = wrist_orient[2] - 90
            wrist_orient = [x * (np.pi / 180) for x in wrist_orient]
           

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
                elbow_velocities[i] = np.round((elbow_positions[i] - elbow_positions[i-1]) / dt, 5)
                wrist_pos_velocities[i] = np.round((wrist_positions[i] - wrist_positions[i-1]) / dt, 5)
                wrist_orient_velocities[i] = np.round((wrist_orientations[i] - wrist_orientations[i-1]) / dt, 5)
        
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
    
    def __iter__(self):
        """
        Make the Parser class iterable.
        Returns one row of trajectory data at a time.
        
        Each iteration returns a dictionary with all data for a single timestep.
        """
        if self.trajectory_data is None:
            print("No trajectory data to iterate. Call parse_trajectory_file() first.")
            return
            
        self._iter_index = 0
        self._iter_max = len(self.trajectory_data['times'])
        return self
    
    def __next__(self):
        """Return the next item in the iteration."""
        if self._iter_index >= self._iter_max:
            raise StopIteration
            
        # Create a dictionary with data for the current timestep
        row_data = {
            'time': self.trajectory_data['times'][self._iter_index],
            'elbow_position': self.trajectory_data['elbow_positions'][self._iter_index],
            'elbow_velocity': self.trajectory_data['elbow_velocities'][self._iter_index],
            'wrist_position': self.trajectory_data['wrist_positions'][self._iter_index],
            'wrist_position_velocity': self.trajectory_data['wrist_pos_velocities'][self._iter_index],
            'wrist_orientation': self.trajectory_data['wrist_orientations'][self._iter_index],
            'wrist_orientation_velocity': self.trajectory_data['wrist_orient_velocities'][self._iter_index]
        }
        
        self._iter_index += 1
        return row_data
    
    def save_to_csv(self):
        """
        Save the parsed trajectory data to a CSV file.
        """
        if self.trajectory_data is None:
            print("No trajectory data to save. Call parse_trajectory_file() first.")
            return False
        
        data = {
            'Time': self.trajectory_data['times']
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


def main():
    parser = Parser()
    parser.parse_trajectory_file()
    
    # parser.save_to_csv()
    
    print("\nПервые 5 точек преобразованной траектории:")
    for i, row in enumerate(parser):
        print(f"Точка {i}:")
        print(f"  Время: {row['time']}")
        print(f"  Локоть (преобразованный): {row['elbow_position']}")
        print(f"  Запястье (преобразованное): {row['wrist_position']}")
        
        if i >= 10:
            break


if __name__ == "__main__":
    main()