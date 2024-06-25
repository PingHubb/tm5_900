import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from scipy.interpolate import PchipInterpolator

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

def exponential_func(x, a, b, c):
    """ Exponential function a * exp(b * x) + c used for fitting. """
    # b = -6 * np.exp(0.0065 * x) + 1700
    # return b

    return a * np.exp(b * x) + c

def polynomial_func(x, a, b, c):
    """ Polynomial function a * x**2 + b * x + c used for fitting. """
    return a * x**2 + b * x + c

def fit_and_plot(data, x_values, title, save_path):
    """ Fits functions to data and plots both the data and the fits, then saves to a file without displaying. """
    plt.figure(figsize=(10, 6))
    plt.plot(x_values, data, 'o', label='Smoothed Data', color='blue')

    try:
        # Fit exponential function
        params_exp, _ = curve_fit(exponential_func, x_values, data, maxfev=1000)
        exp_fit = exponential_func(x_values, *params_exp)
        plt.plot(x_values, exp_fit, label='Exponential Fit', color='red')
        exp_eq = f'y = {params_exp[0]:.2f} * exp({params_exp[1]:.2f} * x) + {params_exp[2]:.2f}'

        # Fit polynomial function
        params_poly, _ = curve_fit(polynomial_func, x_values, data, maxfev=5000)
        poly_fit = polynomial_func(x_values, *params_poly)
        plt.plot(x_values, poly_fit, label='Polynomial Fit', color='green')
        poly_eq = f'y = {params_poly[0]:.2f} * x^2 + {params_poly[1]:.2f} * x + {params_poly[2]:.2f}'

        # Display equations on the plot at a more central position
        plt.text(0.5, 0.5, exp_eq, transform=plt.gca().transAxes, fontsize=9, verticalalignment='center', horizontalalignment='center', bbox=dict(facecolor='white', alpha=0.8))
        plt.text(0.5, 0.45, poly_eq, transform=plt.gca().transAxes, fontsize=9, verticalalignment='center', horizontalalignment='center', bbox=dict(facecolor='white', alpha=0.8))

    except RuntimeError as e:
        print(f"Fit failed: {str(e)}")

    plt.title(title)
    plt.xlabel('Time')
    plt.ylabel('Data Value')
    plt.legend()
    plt.grid(True)

    # Save the plot directly without showing it
    plt.savefig(save_path)
    plt.close()  # Close the plot after saving to free up resources


def plot_and_save_data(data, directory, config, label):
    """ Processes and plots data for each moving segment without displaying the plot. """
    for i, segment in enumerate(data):
        smoothed_segment = smooth_data(segment)
        x_values = np.arange(len(smoothed_segment))
        segment_label = f'Moving Down {i + 1}' if i % 2 == 0 else f'Moving Up {i + 1}'
        save_path = os.path.join(directory, config, f'{segment_label}_{label}_fit.png')
        fit_and_plot(smoothed_segment, x_values, f'{segment_label} in {config} {label}', save_path)

def process_all_files(base_directory):
    """ Processes all files in the specified directory. """
    configurations = ['1x1', '1x3', '1x5', '3x1', '3x3', '3x5', '5x1', '5x3', '5x5']
    labels = ['A', 'B', 'C']
    index_map = {'A': 4, 'B': 13, 'C': 22}

    for config in configurations:
        for label in labels:
            index = index_map[label]
            filepath = os.path.join(base_directory, config, f'sensor_readings_{config}_{label}.txt')
            if os.path.exists(filepath):
                data = extract_data_from_file(filepath, index)
                ensure_directory_exists(os.path.join(base_directory, config))
                plot_and_save_data(data, base_directory, config, label)

if __name__ == '__main__':
    base_directory = '/home/ping2/ros2_ws/src/lan_control/lan_control/resources/sensor_data'
    process_all_files(base_directory)
