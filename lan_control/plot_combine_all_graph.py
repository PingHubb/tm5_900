import os
import numpy as np
import matplotlib.pyplot as plt


def extract_data_from_file(filename, index):
    """ Extracts data from a given file using a specific index. """
    data = []
    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()
            if line.startswith('---'):
                continue
            numbers = line.split(', ')
            if len(numbers) > index:
                data.append(float(numbers[index]))
    return np.arange(len(data)), np.array(data)  # Return indices as x, and measurements as y


def plot_all_data(directory):
    """ Plots data from all files in the directory on the same graph. """
    plt.figure(figsize=(12, 8))

    configurations = ['1x1', '1x3', '1x5', '3x1', '3x3', '3x5', '5x1', '5x3', '5x5']
    labels = ['A', 'B', 'C']
    index_map = {'A': 4, 'B': 13, 'C': 22}

    for config in configurations:
        for label in labels:
            index = index_map[label]
            filename = f'sensor_readings_{config}_{label}.txt'
            filepath = os.path.join(directory, config, filename)
            if os.path.exists(filepath):
                xdata, ydata = extract_data_from_file(filepath, index)
                plt.plot(xdata, ydata, label=f'{config}_{label}')

    plt.title('Comparison of Sensor Data Across Configurations and Labels')
    plt.xlabel('Time (index)')
    plt.ylabel('Sensor Value')
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    directory_path = '/home/ping2/ros2_ws/src/lan_control/lan_control/resources/sensor_data'  # Adjust this path to your data directory
    plot_all_data(directory_path)
