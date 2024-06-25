import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


def extract_and_average(filename):
    data = {}
    current_key = None
    current_values = []

    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()
            if line.startswith('---'):
                if current_values:
                    data[current_key] = np.mean(current_values, axis=0)
                current_key = line.strip('--- ')
                current_values = []
            else:
                current_values.append(np.array(list(map(float, line.split(', ')[:-5]))))

        if current_values:
            data[current_key] = np.mean(current_values, axis=0)
    return data


def calculate_differences(data):
    reference_data = data['Reference Data']
    differences = {}
    for key, averages in data.items():
        if key != 'Reference Data':
            differences[key] = averages - reference_data
    return differences


def write_averages(data, output_filename):
    with open(output_filename, 'w') as f:
        for key, averages in data.items():
            f.write(f'{key}\n')
            f.write(', '.join(f'{avg:.4f}' for avg in averages) + '\n')


def write_differences(differences, output_filename):
    with open(output_filename, 'w') as f:
        for key, diff in differences.items():
            f.write(f'--- Difference for {key} ---\n')
            f.write(', '.join(f'{d:.4f}' for d in diff) + '\n')


def extract_required_indices(differences, output_filename):
    indices = {
        'Path: 0': 4, 'Path: 1': 3, 'Path: 2': 2, 'Path: 3': 1,
        'Path: 4': 0, 'Path: 5': 5, 'Path: 6': 6, 'Path: 7': 7, 'Path: 8': 8, 'Path: 9': 9,
        'Path: 10': 14, 'Path: 11': 13, 'Path: 12': 12, 'Path: 13': 11, 'Path: 14': 10,
        'Path: 15': 15, 'Path: 16': 16, 'Path: 17': 17, 'Path: 18': 18, 'Path: 19': 19,
        'Path: 20': 24, 'Path: 21': 23, 'Path: 22': 22, 'Path: 23': 21, 'Path: 24': 20
    }
    selected_differences = {}
    with open(output_filename, 'w') as f:
        for key, index in indices.items():
            if key in differences:
                value = differences[key][index]
                selected_differences[key] = value
                f.write(f'{key} - Index {index}: {value:.4f}\n')
    return selected_differences


def compute_weights(differences):
    # Find the most negative value in the differences
    min_value = min(differences.values())

    weights = {}
    for key, value in differences.items():
        if value != 0:  # To ensure no division by zero
            weights[key] = min_value / value
        else:
            weights[key] = 0  # Handle zero differences to avoid division by zero
    return weights


def write_weights(weights, output_filename):
    with open(output_filename, 'w') as f:
        for key, weight in weights.items():
            f.write(f'{key}: {weight:.4f}\n')


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


def plot_weights_3d_histogram(weights):
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    paths = list(weights.keys())
    weight_values = list(weights.values())

    # Generate grid positions, reversing the order of y positions
    xpos = [i % 5 for i in range(len(paths))]  # X positions from 0 to 4
    ypos = [4 - (i // 5) for i in range(len(paths))]  # Y positions from 4 to 0
    zpos = np.zeros(len(paths))  # Z positions (starts of bars)

    dx = np.ones(len(paths)) * 0.8  # Width of the bars along the X-axis
    dy = np.ones(len(paths)) * 0.8  # Width of the bars along the Y-axis
    dz = weight_values  # Height of the bars along the Z-axis

    # Generate colors from the 'viridis' colormap and adjust for weight = 1 with transparency
    colors = [plt.cm.viridis(value) if value != 1 else (1, 0, 0, 0.5) for value in np.linspace(0, 1, len(paths))]

    for i in range(len(paths)):
        color = colors[i]
        # Apply transparency by setting the alpha value
        ax.bar3d(xpos[i], ypos[i], zpos[i], dx[i], dy[i], dz[i], color=color, alpha=0.8)  # 80% transparency

    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Weights')

    ax.set_xticks(range(5))
    ax.set_yticks(range(5))
    ax.set_xticklabels([f'Col {i+1}' for i in range(5)])
    ax.set_yticklabels([f'Row {5-i}' for i in range(5)])  # Adjust row labels to match the reversed y positions

    plt.title('3D Histogram of Weights with Transparency in a 5x5 Grid')
    plt.grid(True)
    plt.tight_layout()  # Adjust layout
    plt.savefig('/home/ping2/ros2_ws/src/lan_control/lan_control/resources/experiment_2/1.5cm/weights_3d_plot_5x5.png')  # Save the plot as a PNG file
    plt.show()


if __name__ == '__main__':
    filename = '/home/ping2/ros2_ws/src/lan_control/lan_control/resources/experiment_2/1.5cm/sensor_readings_experiment_2_1.5cm.txt'
    differences_output = '/home/ping2/ros2_ws/src/lan_control/lan_control/resources/experiment_2/1.5cm/differences_experiment_2.txt'
    averages_output = '/home/ping2/ros2_ws/src/lan_control/lan_control/resources/experiment_2/1.5cm/averages_experiment_2.txt'
    indices_output = '/home/ping2/ros2_ws/src/lan_control/lan_control/resources/experiment_2/1.5cm/corresponding_index_experiment_2.txt'
    weights_output = '/home/ping2/ros2_ws/src/lan_control/lan_control/resources/experiment_2/1.5cm/weights_experiment_2.txt'

    data_averages = extract_and_average(filename)
    differences = calculate_differences(data_averages)
    write_differences(differences, differences_output)
    write_averages(data_averages, averages_output)
    selected_differences = extract_required_indices(differences, indices_output)
    weights = compute_weights(selected_differences)
    write_weights(weights, weights_output)
    plot_weights_3d_histogram(weights)
