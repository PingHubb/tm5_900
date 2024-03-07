import numpy as np

# def process_data(calibration_values, read_raw):
#     touchDiffData = np.array(calibration_values) - np.array(read_raw)
#     touchDiffPerc = (10000 * touchDiffData) / np.array(calibration_values)
#
#     # Coefficients for linear mapping
#     a, b = 24.9875, -498.75
#
#     # Initialize the dis array
#     dis = np.zeros_like(touchDiffPerc)
#
#     # Conditions for mapping
#     dis[touchDiffPerc < 15] = -8
#     dis[(touchDiffPerc >= 15) & (touchDiffPerc < 20)] = 5
#     dis[(touchDiffPerc >= 20) & (touchDiffPerc < 100)] = a * touchDiffPerc[(touchDiffPerc >= 20) & (touchDiffPerc < 100)] + b
#     dis[(touchDiffPerc >= 100)] = 2000
#
#     return dis


# def process_data(calibration_values, read_raw):
#     touchDiffData = np.array(calibration_values) - np.array(read_raw)
#     touchDiffPerc = (10000 * touchDiffData) / np.array(calibration_values)
#
#     # Coefficients for linear mapping (recalculated)
#     a, b = 25.0, -500.0
#
#     # Initialize the dis array
#     dis = np.zeros_like(touchDiffPerc)
#
#     # Conditions for mapping
#     dis[touchDiffPerc < 20] = -8
#     dis[(touchDiffPerc >= 20) & (touchDiffPerc < 100)] = a * touchDiffPerc[(touchDiffPerc >= 20) & (touchDiffPerc < 100)] + b
#     dis[touchDiffPerc >= 100] = 2000
#
#     return dis


# def process_data(calibration_values, read_raw):
#     touchDiffData = np.array(calibration_values) - np.array(read_raw)
#     touchDiffPerc = (10000 * touchDiffData) / np.array(calibration_values)
#
#     # Coefficients for quadratic mapping (scaling 0 to 1000 from diff 20 to 100)
#     a, b, c = 0.125, -2.5, 0  # c is effectively zero
#
#     # Initialize the dis array
#     dis = np.zeros_like(touchDiffPerc)
#
#     # Conditions for mapping
#     dis[touchDiffPerc < 20] = -8
#     dis[(touchDiffPerc >= 20) & (touchDiffPerc < 100)] = a * touchDiffPerc[(touchDiffPerc >= 20) & (touchDiffPerc < 100)]**2 + b * touchDiffPerc[(touchDiffPerc >= 20) & (touchDiffPerc < 100)] + c
#     dis[touchDiffPerc >= 100] = 1000
#
#     return dis


# def process_data(calibration_values, read_raw):
#     touchDiffData = np.array(calibration_values) - np.array(read_raw)
#     touchDiffPerc = (10000 * touchDiffData) / np.array(calibration_values)
#
#     # Coefficients for quadratic mapping (scaling 0 to 2000 from diff 20 to 100)
#     a, b, c = 0.25, -5.0, 0  # c is effectively zero
#
#     # Initialize the dis array
#     dis = np.zeros_like(touchDiffPerc)
#
#     # Conditions for mapping
#     dis[touchDiffPerc < 20] = -8
#     dis[(touchDiffPerc >= 20) & (touchDiffPerc < 100)] = a * touchDiffPerc[(touchDiffPerc >= 20) & (touchDiffPerc < 100)]**2 + b * touchDiffPerc[(touchDiffPerc >= 20) & (touchDiffPerc < 100)] + c
#     dis[touchDiffPerc >= 100] = 2000
#
#     return dis


def process_data(calibration_values, read_raw):
    touchDiffData = np.array(calibration_values) - np.array(read_raw)
    touchDiffPerc = (10000 * touchDiffData) / np.array(calibration_values)

    # Coefficients for quadratic mapping (scaling 100 to 2000 from diff 20 to 100)
    a, b, c = 0.1875, 1.25, 0  # c is effectively zero

    # Initialize the dis array
    dis = np.zeros_like(touchDiffPerc)

    # Conditions for mapping
    dis[touchDiffPerc < 20] = -8
    dis[(touchDiffPerc >= 20) & (touchDiffPerc < 100)] = a * touchDiffPerc[(touchDiffPerc >= 20) & (touchDiffPerc < 100)]**2 + b * touchDiffPerc[(touchDiffPerc >= 20) & (touchDiffPerc < 100)] + c
    dis[touchDiffPerc >= 100] = 2000

    return dis


