import os
import matplotlib.pyplot as plt

directory_paths = '/home/ping2/ros2_ws/src/lan_control/lan_control/resources/sensor_data'  # Update with the actual path where your files are stored

index_number = 13
label_name = 'B'

def extract_and_normalize_data(filename):
    index = index_number  # For label 'A', using index 4
    data = []
    with open(filename, 'r') as file:
        for line in file:
            if not line.strip().startswith('---'):  # Assuming non-header lines contain data
                parts = line.strip().split(', ')
                if len(parts) > index:  # Check there are enough elements
                    data.append(float(parts[index]))  # Extract the data from the specified index

    if data:
        # Normalize data by subtracting the first value from all elements
        baseline = data[0]
        normalized_data = [x - baseline for x in data]
        return normalized_data
    return data

def plot_normalized_data_for_label(base_directory):
    plt.figure(figsize=(15, 8))
    configurations = ['1x1', '1x3', '1x5', '3x1', '3x3', '3x5', '5x1', '5x3', '5x5']
    label = label_name  # Focus on label

    for config in configurations:
        filename = f'sensor_readings_{config}_{label}.txt'
        filepath = os.path.join(base_directory, filename)
        if os.path.exists(filepath):
            normalized_data = extract_and_normalize_data(filepath)
            plt.plot(normalized_data, label=f'{config}_{label}', marker='o', linestyle='-')

    plt.title(f'Normalized Sensor Data for Label {label_name} Across Different Configurations')
    plt.xlabel('Time in 20Hz intervals')
    plt.ylabel(f'Normalized Sensor Value at {label_name}')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    directory_path = directory_paths  # Update with the actual path where your files are stored
    plot_normalized_data_for_label(directory_path)