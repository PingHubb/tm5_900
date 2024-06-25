import matplotlib.pyplot as plt


def extract_data_from_file(filename):
    data = []
    current_step_data = []
    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()
            if line.startswith('---'):  # Identify a new step
                if current_step_data:  # Save the previous step's data if present
                    data.append(current_step_data)
                    current_step_data = []  # Reset for next step
                continue
            numbers = line.split(', ')
            if len(numbers) > 13:  # Ensure there are enough elements
                current_step_data.append(float(numbers[13]))  # Convert to float and add to list

    if current_step_data:  # Add the last step's data
        data.append(current_step_data)
    return data


def plot_data(data):
    plt.figure(figsize=(10, 6))

    # Initial x-value offset for the first step
    x_offset = 0

    # Define colors and styles for clarity
    colors = ['blue', 'green']  # Extendable for more steps
    markers = ['o', '^']  # Different markers for different steps
    linestyles = ['-', '--']  # Different styles for different types of data

    # Loop through each set of step data
    for i, step_data in enumerate(data):
        x_values = range(x_offset, x_offset + len(step_data))  # Create x-values starting from the last offset
        label = 'Moving Down' if i == 0 else 'Moving Up' if i == 1 else f'Step {i + 1}'

        # Plot using the adjusted x-values
        plt.plot(x_values, step_data, marker=markers[i % len(markers)], linestyle=linestyles[i % len(linestyles)],
                 color=colors[i % len(colors)], label=label)

        # Update the offset for the next step to continue where the last one ended
        x_offset += len(step_data)

    plt.title('Sensor Data at B Across Different Steps')
    plt.xlabel('Time')
    plt.ylabel('Raw data value at B')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    filename = '/home/ping2/ros2_ws/src/lan_control/sensor_readings_B.txt'  # Update with your actual file path
    sensor_data = extract_data_from_file(filename)
    plot_data(sensor_data)