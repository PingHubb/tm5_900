import os
import numpy as np
import matplotlib.pyplot as plt

def ensure_directory_exists(directory):
    """ Ensure the directory exists, create if it does not. """
    if not os.path.exists(directory):
        os.makedirs(directory)

def extract_data_from_file(filename, index):
    """ Extracts data from a given file using a specific index. """
    data = []
    current_step_data = []
    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()
            if line.startswith('---'):  # Check for new data segment
                if current_step_data:
                    data.append(current_step_data)
                    current_step_data = []
                continue
            numbers = line.split(', ')
            if len(numbers) > index:
                current_step_data.append(float(numbers[index]))
    if current_step_data:
        data.append(current_step_data)
    return data

def smooth_data(data, window_size=15):
    """ Applies a moving average to the data with a specified window size. """
    if len(data) < window_size:
        return data
    return np.convolve(data, np.ones(window_size) / window_size, mode='valid')

def plot_and_save_data(data, directory, config, label):
    """ Plots and saves the data to a file within a specific configuration directory. """
    plt.figure(figsize=(10, 6))
    x_offset = 0
    for step_data in data:
        x_values = np.arange(x_offset, x_offset + len(step_data))
        # Plot raw data
        plt.plot(x_values, step_data, 'o', linestyle='-', label='Raw Data', alpha=0.5, color='gray')

        # Smooth data and adjust x_values for smoothed data
        if len(step_data) > 51:
            smoothed_data = smooth_data(step_data, window_size=51)
            # Adjust x_values to center the smoothed data
            x_values_smoothed = np.arange(x_offset + 25, x_offset + 25 + len(smoothed_data))
            plt.plot(x_values_smoothed, smoothed_data, linestyle='-', label='Smoothed Curve', color='blue')
        x_offset += len(step_data)

    print("length of x_values_smoothed_data: ", len(x_values_smoothed))

    plt.title(f'Smoothed Sensor Data - {config} {label}')
    plt.xlabel('Time')
    plt.ylabel('Data Value')
    plt.legend()
    plt.grid(True)
    save_path = os.path.join(directory, config, f'sensor_readings_{config}_{label}_smoothed.png')
    plt.savefig(save_path)
    plt.close()

def process_all_files(base_directory):
    """ Processes all files in the specified directory. """
    configurations = ['1x1', '1x3', '1x5', '3x1', '3x3', '3x5', '5x1', '5x3', '5x5']
    labels = ['A', 'B', 'C']
    index_map = {'A': 4, 'B': 13, 'C': 22}  # Map labels to their respective indices

    for config in configurations:
        for label in labels:
            index = index_map[label]
            filepath = os.path.join(base_directory, config, f'sensor_readings_{config}_{label}.txt')
            if os.path.exists(filepath):
                data = extract_data_from_file(filepath, index)
                ensure_directory_exists(os.path.join(base_directory, config))  # Ensure config directory exists
                plot_and_save_data(data, base_directory, config, label)

if __name__ == '__main__':
    base_directory = '/home/ping2/ros2_ws/src/lan_control/lan_control/resources/sensor_data'
    process_all_files(base_directory)