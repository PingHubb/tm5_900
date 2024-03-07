import numpy as np
def process_data(calibration_values, read_raw):

    touchDiffData = np.array(calibration_values) - np.array(read_raw)
    touchDiffPerc = (10000 * touchDiffData) / np.array(calibration_values)
    # touchDiffPerc = (5000 * touchDiffData) / np.array(calibration_values)
    dis = touchDiffPerc
    dis[touchDiffPerc < 15] = -8
    dis[(touchDiffPerc >= 15) & (touchDiffPerc < 20)] = 5
    dis[(touchDiffPerc >= 20) & (touchDiffPerc < 30)] = 4
    dis[(touchDiffPerc >= 30) & (touchDiffPerc < 50)] = 3
    dis[(touchDiffPerc >= 50) & (touchDiffPerc < 70)] = 2
    dis[(touchDiffPerc >= 70)] = 1

    return dis
