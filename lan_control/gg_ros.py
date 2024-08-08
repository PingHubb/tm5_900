import arcade
import csv
from lan_control.OLD import sensor_serial_api_old
import polyscope as ps
from multiprocessing import Process, Queue, RawArray, Lock
import glob
import os
import openmesh as om
import rclpy
from rclpy.node import Node
import numpy as np
import math
import threading
import time
from math import cos, sin, radians
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String
from tm_msgs.srv import SetEvent, SetPositions
from sensor_msgs.msg import JointState

""" For SENSOR UI """
# Serial Communication Setup
serial_port = '/dev/ttyACM0'
baud_rate = 9600

# Constants for the UI
SCREEN_WIDTH = 1200  # 1100
SCREEN_HEIGHT = 900  # 700
BUTTON_WIDTH = 250
BUTTON_HEIGHT = 50
BUTTON_X_POSITIONS = SCREEN_WIDTH / 5  # Adjust based on your layout
BUTTON_Y_POSITIONS = [800, 600, 530, 460, 390, 320, 250, 150, 100]  # Adjust based on your layout and number of buttons
NEON_COLOR = (58, 255, 217)  # A neon-like color
HOVER_COLOR = (255, 235, 59)  # Color when hovered

number_of_data = 50
threshold_offset = 10
# threshold_offset = 12   #  < --------- this is stable

remove_last_how_many_value = -10  # Remove the last 10 values from the heatmap data
useless_row = -999  # Row to skip in the heatmap
useless_col = -999  # Column to skip in the heatmap

""" For POLYSCOPE UI """
zeta = 1.0
d = 0.01
rho = 0.1
eta = 0.0001

goal_positions = [1.0, -4.88533739e-01, 1.57, -1.09090181e+00 + math.pi / 2, 1.57, 0.0]
# goal_positions = [math.radians(-180), math.radians(45), math.radians(45), math.radians(0), math.radians(90), math.radians(0)]
# goal_positions = [0.0, 0.0, 1.58, 0.0, 1.58, 0.0]

""" Sensor position parameters"""
# sensor_angle = np.linspace(40, 160, 10)  # Adjusting to ensure a semicircle or your specific arc
# y_position = np.linspace(-0.0405, 0.0405, 10)  # Generates 10 points directly
# radius = 0.0906 / 2
sensor_angle = np.linspace(0, 180, 10)
y_position = np.linspace(-0.08, 0.08, 10)
radius = 0.3 / 2

""" UI SENSOR BELOW"""
class NeonButton:
    """ Button with a neon-style effect """
    def __init__(self, center_x, center_y, width, height, text, action_function):
        self.center_x = center_x
        self.center_y = center_y
        self.width = width
        self.height = height
        self.text = text
        self.action_function = action_function
        self.is_hovered = False

    def draw(self):
        # Outer glow for neon effect
        outer_color = HOVER_COLOR if self.is_hovered else NEON_COLOR
        for i in range(10, 0, -1):  # Create a glow effect
            alpha = int(255 * (i / 10))
            color = (*outer_color, alpha)
            arcade.draw_rectangle_filled(self.center_x, self.center_y, self.width + i*2, self.height + i*2, color)

        # Button rectangle
        arcade.draw_rectangle_filled(self.center_x, self.center_y, self.width, self.height, arcade.color.BLACK)

        # Button text
        arcade.draw_text(self.text, self.center_x, self.center_y, NEON_COLOR, font_size=20, anchor_x="center", anchor_y="center")

    def check_click(self, x, y):
        if (self.center_x - self.width / 2 <= x <= self.center_x + self.width / 2 and
                self.center_y - self.height / 2 <= y <= self.center_y + self.height / 2):
            print(f"Button '{self.text}' clicked")  # Print the button text
            self.action_function()

    def update_hover(self, x, y):
        self.is_hovered = (self.center_x - self.width / 2 <= x <= self.center_x + self.width / 2 and
                           self.center_y - self.height / 2 <= y <= self.center_y + self.height / 2)


class MyGame(arcade.Window):
    def __init__(self, shared_data, lock, width, height, title, commander):
        super().__init__(width, height, title)
        background_path = os.path.expanduser("~/Downloads/nature.jpg")  # Expand the path to the full directory
        self.shared_data = shared_data
        self.lock = lock
        self.commander = commander
        self.button_list = []
        self.setup_buttons()
        self.background = arcade.load_texture(background_path)  # Load your background image
        self.automation_active = False  # Flag to control automation
        self.arrow_visible = True
        self.arrow_timer = 0
        self.sequence_active = False
        self.sequence_step = 0
        self.counter = 0
        self.heatmap_data = None  # Initialize with zeros or appropriate default values
        self.data = None
        self.calibration_data = None
        self.channel_check = None
        self.show_heatmap = False  # Flag to control heatmap visibility
        self.raw_data = None
        self.is_recording = False  # Add this line to track recording state
        self.selected_cell = None
        self.matrix_height = 0
        self.matrix_width = 0
        self.show_graph = False  # Add this line
        self.y_axis_center_value = None
        self.smoothed_curve_data = {}
        self.initial_threshold_calculated = {}
        self.threshold_diff = {}
        self.threshold_max = {}
        self.threshold_min = {}
        self.filtered_data = {}
        self.graph_background_color = arcade.color.WHITE_SMOKE  # Default graph background color
        self.threshold_triggered = {}  # To track if the threshold condition is met
        self.current_threshold_max = {}
        self.current_threshold_min = {}

        self.processed_data = {}  # Stores processed data for all cells
        self.temp_smoothed_data = {}
        self.process_all = False
        self.initial_processing_done = False
        self.cells_processed_initially = set()
        self.red_intensity_dict = {}  # Initialize the dictionary to store red intensity values
        self.above_threshold_triggered = False
        self.below_threshold_triggered = False

    def start_sequence(self):
        self.sequence_active = True
        self.sequence_step = 0
        self.process_all = True  # Process data for all cells when "Start" is pressed

    def draw_arrow(self, x, y, direction):
        """ Draw a simple arrow pointing to the given coordinates. """
        if direction == "right":
            # Arrow pointing to the left
            arcade.draw_line(x + 20, y, x - 20, y, arcade.color.RED, 4)
            arcade.draw_line(x - 20, y, x - 10, y - 10, arcade.color.RED, 4)
            arcade.draw_line(x - 20, y, x - 10, y + 10, arcade.color.RED, 4)
        elif direction == "left":
            # Arrow pointing to the right
            arcade.draw_line(x - 20, y, x + 20, y, arcade.color.RED, 4)
            arcade.draw_line(x + 20, y, x + 10, y - 10, arcade.color.RED, 4)
            arcade.draw_line(x + 20, y, x + 10, y + 10, arcade.color.RED, 4)

    def test_button_function(self):
        print("Recalculating thresholds for all cells...")
        self.recalculate_thresholds_for_all_cells()
        print("Threshold recalculation complete.")

    def recalculate_thresholds_for_all_cells(self):
        for row in range(self.matrix_height):
            for col in range(self.matrix_width):
                cell_key = (row, col)
                if cell_key in self.processed_data:
                    # Recalculate thresholds using existing processed data for the cell
                    data = self.processed_data[cell_key]
                    if data:  # Ensure there is data to process
                        self.initial_threshold_calculated[(row, col)] = False  # Reset the flag
                        self.set_thresholds(row, col, data)

    def perform_channel_check(self):
        self.channel_check = self.commander.channel_check()
        print(f"ChannelDriver: {self.channel_check[0]} ChannelSensor: {self.channel_check[1]}")

    def perform_update_cal(self):
        self.calibration_data = self.commander.update_cal()
        # print(f"Calibration Data is equal to: {self.calibration_data}")

    def perform_read_raw(self):
        self.raw_data = self.commander.read_raw()
        # print(f"{self.counter} Raw Data is equal to : {self.raw_data}")

    def toggle_heatmap(self):
        self.show_heatmap = not self.show_heatmap
        print(f"Heatmap visibility set to: {self.show_heatmap}")

    def toggle_automation(self):
        # Toggle the automation_active flag
        self.automation_active = not self.automation_active
        if self.automation_active:
            print("Automation Started")
        else:
            print("Automation Stopped")

    def record_heatmap_data(self):
        self.is_recording = not self.is_recording  # Toggle recording state
        if self.is_recording:
            print("Recording started.")
        else:
            print("Recording stopped.")

    def record_to_csv(self):
        filename = "OLD/heatmap_data.csv"
        with open(filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(self.heatmap_data)

    def setup_buttons(self):
        # Setup buttons with neon effect
        button_commands = [
            ("Test", self.test_button_function),
            ("Channel Check", self.perform_channel_check),
            ("Update Cal", self.perform_update_cal),
            ("Read Cal", self.commander.read_cal),
            ("Read Raw", self.perform_read_raw),
            ("Automation", self.toggle_automation),
            ("Heatmap", self.toggle_heatmap),
            ("Record", self.record_heatmap_data),
            ("Start", self.start_sequence),
        ]
        for i, (title, action) in enumerate(button_commands):
            self.button_list.append(NeonButton(BUTTON_X_POSITIONS, BUTTON_Y_POSITIONS[i], BUTTON_WIDTH, BUTTON_HEIGHT, title, action))

    def on_update(self, delta_time):
        # If automation is active, continuously call commander.read_raw

        self.time_count = time.time()

        if self.automation_active:
            self.counter += 1
            self.perform_read_raw()
            if self.raw_data and self.calibration_data:
                # Update the heatmap data
                self.heatmap_data = self.raw_data[:remove_last_how_many_value]  # <---------- Test for raw data
                # print("length of heatmap data: ", len(self.heatmap_data))

                """ For switching col to row major"""
                # # Create an empty matrix with the desired dimensions
                # row_major_matrix = [[None for _ in range(10)] for _ in range(11)]
                # # Populate the matrix in row-major order
                # for index, value in enumerate(self.heatmap_data):
                #     row = index % 11
                #     col = index // 11
                #     row_major_matrix[row][col] = value
                # flattened_data = [item for sublist in row_major_matrix for item in sublist]
                """ For switching col to row major"""

                # self.ros_publisher.publish_data(flattened_data)
                # print("Length of heatmap data: ", len(flattened_data))
                # print("Heatmap data: ", flattened_data)

        # Blinking arrow logic
        self.arrow_timer += delta_time
        if self.arrow_timer > 0.5:  # Change the interval to speed up or slow down the blink
            self.arrow_timer = 0
            self.arrow_visible = not self.arrow_visible

        # Handle the sequence of actions
        if self.sequence_active:
            if self.sequence_step == 0:
                self.perform_channel_check()
                self.sequence_step += 1
            elif self.sequence_step == 1:
                self.perform_update_cal()
                self.sequence_step += 1
            elif self.sequence_step == 2:
                self.commander.read_cal()
                self.sequence_step += 1
            elif self.sequence_step == 3:
                self.toggle_automation()
                self.sequence_step += 1
            elif self.sequence_step == 4:
                self.toggle_heatmap()
                self.sequence_active = False  # End of sequence

        if self.is_recording and self.heatmap_data is not None:
            self.record_to_csv()

        if self.selected_cell:
            row, col = self.selected_cell
            index = row * self.matrix_width + col
            if index < len(self.heatmap_data):
                print(f"Index: {index}, Row: {row}, Col: {col}, Value: {self.heatmap_data[index]}")

        if self.process_all:
            self.process_all_cells()

        self.update_background_color_for_selected_cell()

    def process_all_cells(self):
        """Preprocess and store data for all cells."""
        if not self.processed_data:  # Initialize if empty
            for row in range(self.matrix_height):
                for col in range(self.matrix_width):
                    self.processed_data[(row, col)] = []
            print(f"Processed data initialized with {self.matrix_height} x {self.matrix_width} cells.")
            print(f"Length of processed data: {len(self.processed_data)}")

        if self.processed_data:
            for row in range(self.matrix_height):
                for col in range(self.matrix_width):
                    cell_index = row * self.matrix_width + col  # matrix_width is 10, because 10 columns
                    if cell_index < len(self.heatmap_data):  # Check if the index is within the data range, which is 110
                        self.process_cell_data(row, col, self.heatmap_data[cell_index])  # Process the data for each cell repeatedly
                    else:
                        print(f"Data for cell ({row}, {col}) is out of range and cannot be processed.")

    def process_cell_data(self, row, col, data):  # Only 1 cell of data is entering here each time
        """Append data for a single cell identified by row and col."""
        self.processed_data[(row, col)].append(data)

        if len(self.processed_data[(row, col)]) > number_of_data:
            self.processed_data[(row, col)].pop(0)
        if len(self.processed_data[(row, col)]) == number_of_data:
            self.set_thresholds(row, col, self.processed_data[(row, col)])
            # Add cell to the tracker
            self.cells_processed_initially.add((row, col))

            # Check if all cells have reached the initial processing stage
            if len(self.cells_processed_initially) == self.matrix_height * self.matrix_width and not self.initial_processing_done:
                print("Start processing the data")
                self.initial_processing_done = True

    def set_thresholds(self, row, col, data):  # "number_of_data" data of 1 cell is entering here
        # Calculate IQR for outlier detection
        q1, q3 = np.percentile(data, [25, 75])
        iqr = q3 - q1
        lower_bound = q1 - 1.5 * iqr
        upper_bound = q3 + 1.5 * iqr

        # Filter out outliers
        # self.filtered_data[(row, col)] = [x for x in data if lower_bound <= x <= upper_bound]   # Remove the outliers
        self.filtered_data[(row, col)] = data

        # Update the smoothed curve with filtered data
        self.smoothed_curve_data[(row, col)] = self.calculate_moving_average(row, col, self.filtered_data[(row, col)])

        # Calculate the initial threshold
        if (row, col) not in self.threshold_max or not self.initial_threshold_calculated[(row, col)]:
            self.threshold_max[(row, col)] = max(self.filtered_data[(row, col)]) + threshold_offset
            self.threshold_min[(row, col)] = min(self.filtered_data[(row, col)]) - threshold_offset
            self.threshold_diff[(row, col)] = self.threshold_max[(row, col)] - self.threshold_min[(row, col)]
            # print(f"Max and Min thresholds for cell ({row}, {col}): {self.threshold_max[(row, col)]}, {self.threshold_min[(row, col)]}")

            # Set flag that initial thresholds are calculated
            self.initial_threshold_calculated[(row, col)] = True

    def calculate_moving_average(self, row, col, data, window_size=10):   # "number_of_data - filtered out data" of 1 cell is entering here
        # Calculate the moving average for the given data
        if (row, col) not in self.temp_smoothed_data:
            self.temp_smoothed_data[(row, col)] = []

        # Calculate moving averages and store them in a temporary list
        temp_smoothed_data = []
        for i in range(len(data)):
            window = data[max(0, i - window_size + 1):i + 1]  # Define the window for the moving average
            temp_smoothed_data.append(np.mean(window))

        # Assign the calculated moving averages directly to ensure the lengths match
        self.temp_smoothed_data[(row, col)] = temp_smoothed_data

        # Return the updated smoothed data for this cell
        return self.temp_smoothed_data[(row, col)]

    def check_heatmap_click(self, x, y):
        square_size = 60
        # These starting positions should match those used in draw_heatmap
        start_x = SCREEN_WIDTH / 2 - (square_size * self.matrix_height) / 2 + 200
        start_y = SCREEN_HEIGHT / 2 - (square_size * self.matrix_width) / 2

        cell_selected = False  # Track if a cell gets selected in this method call

        for row in range(self.matrix_height):
            for col in range(self.matrix_width):
                # Adjust for the 180-degree rotation, similar to the adjustments in draw_heatmap
                cell_x = start_x + row * square_size - square_size / 2  # Center of cell horizontally
                cell_y = start_y + (
                            self.matrix_width - 1 - col) * square_size - square_size / 2  # Center of cell vertically

                # Check if the click is within the bounds of the cell
                if (cell_x <= x <= cell_x + square_size and
                        cell_y <= y <= cell_y + square_size):
                    # Toggle selection state for the clicked cell
                    if self.selected_cell == (row, col):
                        self.selected_cell = None  # Deselect if already selected
                    else:
                        self.selected_cell = (row, col)  # Select the cell
                    cell_selected = True
                    break  # Cell found, no need to continue checking
            if cell_selected:
                break  # Exit the loop early since we've handled the click

        # Deselect if clicked outside any cell
        if not cell_selected and self.selected_cell is not None:
            self.selected_cell = None

    def check_threshold_condition(self):

        for row in range(self.matrix_height):
            for col in range(self.matrix_width):
                cell_key = (row, col)

                # Assume the cell has not triggered the threshold initially
                self.threshold_triggered[cell_key] = False

                if cell_key in self.smoothed_curve_data and len(self.smoothed_curve_data[cell_key]) >= 2:
                    last_data = self.smoothed_curve_data[cell_key][-2:]
                    above_threshold = all(d > self.threshold_max[cell_key] for d in last_data)
                    below_threshold = all(d < self.threshold_min[cell_key] for d in last_data)

                    if above_threshold:
                        self.threshold_triggered[cell_key] = True
                        self.above_threshold_triggered = True
                        self.below_threshold_triggered = False
                    elif below_threshold:
                        self.threshold_triggered[cell_key] = True
                        self.below_threshold_triggered = True
                        self.above_threshold_triggered = False
                    elif above_threshold is False and below_threshold is False:
                        self.below_threshold_triggered = False
                        self.above_threshold_triggered = False

                    # print("above:", self.above_threshold_triggered)
                    # print("below:", self.below_threshold_triggered)

    def update_background_color_for_selected_cell(self):

        if self.selected_cell:  # Ensure there is a selected cell
            cell_key = self.selected_cell
            # Check if the selected cell has triggered the threshold
            if cell_key in self.threshold_triggered and self.threshold_triggered[cell_key]:
                # Check whether it's above or below the threshold
                if len(self.smoothed_curve_data[cell_key]) >= 2:
                    last_data = self.smoothed_curve_data[cell_key][-2:]
                    above_threshold = all(d > self.threshold_max[cell_key] for d in last_data)
                    below_threshold = all(d < self.threshold_min[cell_key] for d in last_data)

                    if above_threshold:
                        self.graph_background_color = arcade.color.GREEN  # Selected cell above threshold
                    elif below_threshold:
                        self.graph_background_color = arcade.color.BLACK  # Selected cell below threshold
                    else:
                        self.graph_background_color = arcade.color.WHITE_SMOKE  # Selected cell within thresholds
            else:
                self.graph_background_color = arcade.color.WHITE_SMOKE  # Selected cell has no threshold data or isn't triggered
        else:
            self.graph_background_color = arcade.color.WHITE_SMOKE  # No cell selected

    def on_draw(self):
        arcade.start_render()
        # Draw the background
        arcade.draw_lrwh_rectangle_textured(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, self.background)  # Draw the background

        # Draw the buttons
        for button in self.button_list:
            button.draw()

        # Draw the arrows (if visible)
        if self.arrow_visible:
            automation_button_y = BUTTON_Y_POSITIONS[-1]
            # Adjust X positions as needed
            self.draw_arrow(BUTTON_X_POSITIONS - 150, automation_button_y, "left")
            self.draw_arrow(BUTTON_X_POSITIONS + 150, automation_button_y, "right")

        # Draw the heatmap if the flag is True
        if self.show_heatmap:
            self.draw_heatmap()

        self.draw_realtime_curve()
        self.check_threshold_condition()

    def on_mouse_motion(self, x, y, dx, dy):
        for button in self.button_list:
            button.update_hover(x, y)

    def on_mouse_press(self, x, y, button, modifiers):
        if button == arcade.MOUSE_BUTTON_LEFT:
            for button in self.button_list:
                button.check_click(x, y)
            if self.show_heatmap:
                self.check_heatmap_click(x, y)

        # Right click
        elif button == arcade.MOUSE_BUTTON_RIGHT:
            self.show_graph = not self.show_graph  # Toggle the visibility of the graph
            self.show_heatmap = not self.show_heatmap  # Toggle the visibility of the heatmap
            if self.show_graph and self.selected_cell:
                row, col = self.selected_cell
                index = row * self.matrix_width + col
                if index < len(self.heatmap_data):
                    self.y_axis_center_value = self.heatmap_data[index]

    def draw_curve(self, data, color, y_min, y_max, y_scale, graph_left, graph_top, graph_height, graph_width):
        if not data:
            return

        x_scale = graph_width / len(data)
        for i in range(1, len(data)):
            x1 = graph_left + (i - 1) * x_scale
            y1 = graph_top - graph_height + (data[i - 1] - y_min) * y_scale
            x2 = graph_left + i * x_scale
            y2 = graph_top - graph_height + (data[i] - y_min) * y_scale
            arcade.draw_line(x1, y1, x2, y2, color, 2)

    def draw_threshold_line(self, value, color, y_min, y_max, y_scale, graph_top, graph_height, graph_left, graph_width):
        # Logic to draw a horizontal line across the graph at 'value' height
        y = graph_top - graph_height + (value - y_min) * y_scale
        arcade.draw_line(graph_left, y, graph_left + graph_width, y, color, 2)

    def draw_realtime_curve(self):
        if not self.selected_cell or not self.processed_data or not self.show_graph:
            return

        row, col = self.selected_cell

        # Configuration for graph dimensions
        graph_left = SCREEN_WIDTH * 0.4
        graph_top = SCREEN_HEIGHT * 0.9
        graph_width = SCREEN_WIDTH * 0.55
        graph_height = SCREEN_HEIGHT * 0.8

        # Draw background for the graph
        arcade.draw_rectangle_filled(graph_left + graph_width / 2, graph_top - graph_height / 2, graph_width,
                                     graph_height, self.graph_background_color)

        # Define y-axis range based on the center value
        if self.y_axis_center_value is not None:
            y_min = self.y_axis_center_value - 100
            y_max = self.y_axis_center_value + 100
        else:
            y_min = min(self.processed_data[(row, col)], default=0) - 100
            y_max = max(self.processed_data[(row, col)], default=0) + 100

        x_scale = graph_width / len(self.processed_data[(row, col)])
        y_scale = graph_height / (y_max - y_min)

        # Draw the graph lines
        for i in range(1, len(self.processed_data[(row, col)])):
            x1 = graph_left + (i - 1) * x_scale
            y1 = graph_top - graph_height + (self.processed_data[(row, col)][i - 1] - y_min) * y_scale
            x2 = graph_left + i * x_scale
            y2 = graph_top - graph_height + (self.processed_data[(row, col)][i] - y_min) * y_scale
            arcade.draw_line(x1, y1, x2, y2, arcade.color.BLUE, 2)

        if self.smoothed_curve_data[(row, col)] and self.initial_threshold_calculated:

            # Draw the threshold lines
            self.draw_threshold_line(self.threshold_max[(row, col)], arcade.color.BROWN, y_min, y_max, y_scale, graph_top, graph_height, graph_left, graph_width)
            self.draw_threshold_line(self.threshold_min[(row, col)], arcade.color.BROWN, y_min, y_max, y_scale, graph_top, graph_height, graph_left, graph_width)

        # Draw original and smoothed curves
        self.draw_curve(self.processed_data[(row, col)], arcade.color.BLUE, y_min, y_max, y_scale, graph_left, graph_top,
                        graph_height, graph_width)
        self.draw_curve(self.smoothed_curve_data[(row, col)], arcade.color.RED, y_min, y_max, y_scale, graph_left, graph_top,
                        graph_height, graph_width)

        # Draw Y-axis
        arcade.draw_line(graph_left, graph_top - graph_height, graph_left, graph_top, arcade.color.BLACK, 2)

        # Draw Y-axis labels
        y_label_interval = 40  # Set the interval for the y-axis labels
        for value in range(y_min, y_max + y_label_interval, y_label_interval):
            y = graph_top - graph_height + (value - y_min) * y_scale
            arcade.draw_text(f"{value}", graph_left - 40, y, arcade.color.BLACK, font_size=10, anchor_x="right")

    def draw_heatmap(self):
        global useless_row, useless_col

        # Min and Max values for the heatmap
        min_value = 0
        max_value = 100  # Adjust the max_value to match the typical range of your data
        max_value_above = 1500
        self.matrix_height = self.channel_check[0] - 1  # Height of the matrix
        self.matrix_width = self.channel_check[1]  # Width of the matrix
        square_size = 60  # Size of each square in the heatmap

        # Adjust starting positions for an additional 90 degrees, total 180 from original
        start_x = SCREEN_WIDTH / 2 - (square_size * self.matrix_height) / 2 + 200
        start_y = SCREEN_HEIGHT / 2 - (square_size * self.matrix_width) / 2

        for row in range(self.matrix_height):
            if row == useless_row:
                continue
            for col in range(self.matrix_width):
                if col == useless_col:
                    continue

                x = start_x + row * square_size
                y = start_y + (self.matrix_width - 1 - col) * square_size

                if (row, col) in self.smoothed_curve_data and self.smoothed_curve_data[(row, col)]:
                    value = self.smoothed_curve_data[(row, col)][-1]
                else:
                    value = 0

                if self.selected_cell == (row, col):
                    color = arcade.color.BLUE
                elif self.threshold_triggered.get((row, col), False):
                    last_data = self.smoothed_curve_data[(row, col)][-2:]
                    above_threshold = all(d > self.threshold_max[(row, col)] for d in last_data)
                    below_threshold = all(d < self.threshold_min[(row, col)] for d in last_data)

                    if above_threshold:
                        difference = value - self.threshold_max[(row, col)]
                        blue_intensity = int(((difference - min_value) / (max_value_above - min_value)) * 255)
                        blue_intensity = max(0, min(255, blue_intensity))  # Ensure blue_intensity is within 0-255
                        color = (255 - blue_intensity, 255 - blue_intensity, 255)
                        self.red_intensity_dict[(row, col)] = -abs(
                            blue_intensity)  # Store the negative intensity for above threshold
                        # print("ROWCOL", row, col, self.red_intensity_dict[(row, col)])
                    elif below_threshold:
                        difference = self.threshold_min[(row, col)] - value
                        red_intensity = int(((difference - min_value) / (max_value - min_value)) * 255)
                        red_intensity = max(0, min(255, red_intensity))  # Ensure red_intensity is within 0-255
                        self.red_intensity_dict[(row, col)] = red_intensity  # Store the intensity in the dictionary
                        color = (255, 255 - red_intensity, 255 - red_intensity)
                    else:
                        color = arcade.color.LIGHT_GRAY
                        self.red_intensity_dict[(row, col)] = 0  # Store 0 intensity for cells not meeting threshold
                else:
                    color = arcade.color.LIGHT_GRAY
                    self.red_intensity_dict[(row, col)] = 0  # Store 0 intensity for cells not meeting threshold

                arcade.draw_rectangle_filled(x, y, square_size, square_size, color)
                text_color = arcade.color.BLACK

                def update_robot_model(self):
                    # for idx, i in enumerate(self.robotModel):
                    #     i.points = self.origin_list[idx]

                    self.T01 = np.array([[cos(self.joints[0]), -sin(self.joints[0]), 0, 0],
                                         [sin(self.joints[0]), cos(self.joints[0]), 0, 0],
                                         [0, 0, 1, 0.1452],
                                         [0, 0, 0, 1]])
                    self.T12 = np.array([[sin(self.joints[1]), cos(self.joints[1]), 0, 0],
                                         [0, 0, 1, 0],
                                         [cos(self.joints[1]), -sin(self.joints[1]), 0, 0],
                                         [0, 0, 0, 1]])
                    self.T23 = np.array([[cos(self.joints[2]), -sin(self.joints[2]), 0, 0.429],
                                         [sin(self.joints[2]), cos(self.joints[2]), 0, 0],
                                         [0, 0, 1, 0],
                                         [0, 0, 0, 1]])
                    self.T34 = np.array([[cos(np.pi / 2 + self.joints[3]), -sin(np.pi / 2 + self.joints[3]), 0, 0.4115],
                                         [sin(np.pi / 2 + self.joints[3]), cos(np.pi / 2 + self.joints[3]), 0, 0],
                                         [0, 0, 1, -0.1223],
                                         [0, 0, 0, 1]])
                    self.T45 = np.array([[cos(self.joints[4]), -sin(self.joints[4]), 0, 0],
                                         [0, cos(np.pi / 2), -sin(np.pi / 2), -0.106],
                                         [sin(self.joints[4]), cos(self.joints[4]), 0, 0],
                                         [0, 0, 0, 1]])
                    self.T56 = np.array([[cos(self.joints[5]), -sin(self.joints[5]), 0, 0],
                                         [0, cos(np.pi / 2), -sin(np.pi / 2), -0.11315],
                                         [sin(self.joints[5]), cos(self.joints[5]), 0, 0],
                                         [0, 0, 0, 1]])

                    self.robotModel[1].transform(self.T01 @ self.reT[0])
                    self.robotModel[2].transform(self.T01 @ self.T12 @ self.reT[1])
                    self.robotModel[3].transform(self.T01 @ self.T12 @ self.T23 @ self.reT[2])
                    self.robotModel[4].transform(self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.reT[3])
                    self.robotModel[5].transform(self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45 @ self.reT[4])
                    self.robotModel[6].transform(
                        self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45 @ self.T56 @ self.reT[5])
                    self.robotModel[7].transform(
                        self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45 @ self.T56 @ self.reT[6])

                    self.reT[0] = np.linalg.inv(self.T01)
                    self.reT[1] = np.linalg.inv(self.T01 @ self.T12)
                    self.reT[2] = np.linalg.inv(self.T01 @ self.T12 @ self.T23)
                    self.reT[3] = np.linalg.inv(self.T01 @ self.T12 @ self.T23 @ self.T34)
                    self.reT[4] = np.linalg.inv(self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45)
                    self.reT[5] = np.linalg.inv(self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45 @ self.T56)
                    self.reT[6] = np.linalg.inv(self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45 @ self.T56)
                # arcade.draw_text(f"{row}, {col}", x, y, text_color, 12, width=square_size, align="center", anchor_x="center",
                #                  anchor_y="center")

        # Assuming you want to use 0 as the default red intensity for missing cells
        red_intensity_list = [self.red_intensity_dict.get((row, col), 0) for row in range(self.matrix_height) for col in
                              range(self.matrix_width) if
                              (row, col) in self.red_intensity_dict or (row != useless_row and col != useless_col)]

        """ For testing the ROS publisher"""
        jj_test = red_intensity_list.copy()  # dim: 110
        jj_text_100 = jj_test[:100]  # dim: 100

        if self.lock:
            for i in range(len(jj_text_100)):
                self.shared_data[i] = jj_text_100[i]
        # print("Shared data: ", self.shared_data)
        """ For testing the ROS publisher"""


""" Testing_polyscope Below"""
class ArtificialPotentialField:
    def __init__(self, joint_position_instance, shared_data, lock):
        self.joint_position_instance = joint_position_instance
        self.shared_data = shared_data
        self.lock = lock
        self.Frep = None
        self.T01 = None
        self.T12 = None
        self.T23 = None
        self.T34 = None
        self.T45 = None
        self.T56 = None
        self.T02 = None
        self.T03 = None
        self.T04 = None
        self.T05 = None
        self.T06 = None
        self.J = None
        self.id_jac = None
        self.delta = None
        self.current_positions_variable = None
        self.goal_positions_variable = None

    def transformation_matrix(self, angle):
        self.T01 = np.array([[cos(angle[0]), -sin(angle[0]), 0, 0],
                             [sin(angle[0]), cos(angle[0]), 0, 0],
                             [0, 0, 1, 0.1452],
                             [0, 0, 0, 1]])
        self.T12 = np.array([[sin(angle[1]), cos(angle[1]), 0, 0],
                             [0, 0, 1, 0],
                             [cos(angle[1]), -sin(angle[1]), 0, 0],
                             [0, 0, 0, 1]])
        self.T23 = np.array([[cos(angle[2]), -sin(angle[2]), 0, 0.429],
                             [sin(angle[2]), cos(angle[2]), 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
        self.T34 = np.array([[cos(np.pi / 2 + angle[3]), -sin(np.pi / 2 + angle[3]), 0, 0.4115],
                             [sin(np.pi / 2 + angle[3]), cos(np.pi / 2 + angle[3]), 0, 0],
                             [0, 0, 1, -0.1223],
                             [0, 0, 0, 1]])
        self.T45 = np.array([[cos(angle[4]), -sin(angle[4]), 0, 0],
                             [0, cos(np.pi / 2), -sin(np.pi / 2), -0.106],
                             [sin(angle[4]), cos(angle[4]), 0, 0],
                             [0, 0, 0, 1]])
        self.T56 = np.array([[cos(angle[5]), -sin(angle[5]), 0, 0],
                             [0, cos(np.pi / 2), -sin(np.pi / 2), -0.11315],
                             [sin(angle[5]), cos(angle[5]), 0, 0],
                             [0, 0, 0, 1]])

        self.T02 = self.T01 @ self.T12
        self.T03 = self.T02 @ self.T23
        self.T04 = self.T03 @ self.T34
        self.T05 = self.T04 @ self.T45
        self.T06 = self.T05 @ self.T56  # dim = 4x4
        self.J = self.jacobian()

    def jacobian(self):
        # Initialize a 6x6 Jacobian matrix with zeros
        J = np.zeros((6, 6))

        # The position of the end-effector is extracted from the last column of the T06 matrix
        end_effector_pos = self.T06[:3, 3]

        # A list of transformation matrices leading up to each joint
        # np.eye(4) represents the identity matrix for the base frame
        transforms = [np.eye(4), self.T01, self.T02, self.T03, self.T04, self.T05]

        for i in range(6):
            # Extract the position of the current joint from the transformation matrix
            joint_pos = transforms[i][:3, 3]

            # Extract the z-axis direction vector of the current joint from the transformation matrix
            z_axis = transforms[i][:3, 2]

            # Compute the linear velocity component for the Jacobian
            # It's calculated as the cross product of the z-axis direction vector and the vector from the joint to the end-effector
            linear_velocity_component = np.cross(z_axis, end_effector_pos - joint_pos)

            # The angular velocity component for the Jacobian is directly the z-axis direction vector for revolute joints
            angular_velocity_component = z_axis

            # Update the Jacobian matrix for the current joint
            # Linear velocity components go to the top 3 rows
            J[:3, i] = linear_velocity_component
            # Angular velocity components go to the bottom 3 rows
            J[3:, i] = angular_velocity_component

        return J

    def get_jacobian_by_index(self, jac, idx):  # only a subset of the robot's joints (up to the specified index) is considered for some calculation or analysis.
        # Create a zero matrix with the same dimensions as the Eigen::MatrixXd::Zero(3, 6)
        self.id_jac = np.zeros((3, 6))

        # Copy elements up to the given index
        for i in range(idx + 1):
            self.id_jac[0, i] = jac[0, i]
            self.id_jac[1, i] = jac[1, i]
            self.id_jac[2, i] = jac[2, i]

        return self.id_jac

    def diff_norm(self, current_positions, goal_positions):
        # Convert lists to numpy arrays if they aren't already
        self.current_positions_variable = np.asarray(current_positions)
        self.goal_positions_variable = np.asarray(goal_positions)

        # Calculate the L2 norm (Euclidean distance)
        self.l2_sum = np.sum((self.current_positions_variable - self.goal_positions_variable) ** 2)
        return np.sqrt(self.l2_sum)

    def norm(self, q):
        l2_sum = sum(x ** 2 for x in q)
        return math.sqrt(l2_sum)

    def normalize(self, q, scale=1.0):
        normal = self.norm(q)
        if normal == 0:
            return q  # Return the original vector if its norm is 0 to avoid division by zero
        return [x / normal * scale for x in q]

    def matrix_row_norm(self, mat):
        self.total_norm = 0
        for row in mat:
            row_norm = sum([element ** 2 for element in row]) ** 0.5
            self.total_norm += row_norm
        return self.total_norm

    def calculate_repulsive_force(self):
        # Rotation matrix for 20 degrees anticlockwise around the Z-axis
        theta = np.deg2rad(20)  # Convert 20 degrees to radians
        R_z = np.array([[np.cos(theta), -np.sin(theta), 0],
                        [np.sin(theta), np.cos(theta),  0],
                        [0,             0,              1]])
        R_z_4x4 = np.eye(4)
        R_z_4x4[:3, :3] = R_z  # Embed the original R_z into the top-left
        # Rotation matrix around Y-axis
        R_y = np.array([[np.cos(theta), 0, np.sin(theta)],
                        [0, 1, 0],
                        [-np.sin(theta), 0, np.cos(theta)]])

        # Rotation matrix around X-axis
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(theta), -np.sin(theta)],
                        [0, np.sin(theta), np.cos(theta)]])

        self.Frep = np.zeros((6, 3))  # Create a matrix of zeros

        # sensor_info = self.joint_position_instance.sensor_signal  # Accessing the sensor_signal attribute
        sensor_info = self.shared_data[:100]

        link5_transform = self.T05

        self.delta = np.zeros(3)  # Assuming delta is a 3D vector

        for i in range(len(sensor_info)):
            if i == 3 or i == 13 or i == 23 or i == 33 or i == 43 or i == 53 or i == 63 or i == 73 or i == 83 or i == 93:
                continue
            if sensor_info[i] > 20:
                local_point = self.joint_position_instance.initial_sensor_points[i]  # dim: 3x1
                local_translate = np.eye(4)  # np.eye[4] creates a 4x4 identity matrix
                local_translate[:3, 3] = local_point  # select the first 3 rows and the 4th column and assign local_point to it

                local_normal = self.normalize(np.array([local_point[0], local_point[1], local_point[2]]))  # Normalize the local_normal vector
                # print("Local normal: ", local_normal)
                # local_normal = local_normal * sensor_info[i] * 0.01 / 255

                local_offset = np.eye(4)  # dim: 4x4
                local_offset[:3, 3] = local_normal  # dim: 3x1 # local_offset = distance from sensor to surface

                mapped_point_transform = link5_transform @ local_translate @ local_offset
                # mapped_point_rotated = mapped_point_transform @ R_z_4x4
                mapped_point = mapped_point_transform[:3, 3]

                surface_point_transform = link5_transform @ local_translate
                # surface_point_rotated = surface_point_transform @ R_z_4x4
                surface_point = surface_point_transform[:3, 3]  # Extracting x, y, z position

                vec = surface_point - mapped_point
                self.delta += vec

            # Calculate the repulsive force based on the delta
        if self.norm(self.delta) < 0.0001:
            pass

        else:
            self.delta = self.normalize(self.delta)
            dist = 0.01  # this seems to be a fixed small distance value
            f = eta * (1 / dist - 1 / rho) * (1 / (dist**2)) * np.array(self.delta)
            self.Frep[4, :] = f  # Assuming that the repulsive force is applied to the 5th row (index 4)

        return self.Frep

    def control_update(self, Frep, Fatt):
        F = Fatt + Frep
        jac = self.jacobian()

        # current_joint_states = self.joint_position_instance.current_positions
        current_joint_states = self.shared_data[100:106]

        control = np.zeros(6) # dim: 6x1

        for i in range(5, 6):  # this loop will iterate exactly once, with i taking the value of 5, over the last joint
            jac_i = self.get_jacobian_by_index(jac, i)
            v = jac_i.T @ Fatt[i, :].T
            control += v

        for i in range(6):
            jac_i = self.get_jacobian_by_index(jac, i)
            v = jac_i.T @ Frep[i, :].T
            control += v

        if self.norm(control) < 0.05 and self.diff_norm(current_joint_states, goal_positions) > 1.0:
            # Log local minimum correction
            for j in range(6):
                control[j] += (goal_positions[j] - current_joint_states[j]) * 0.01

        return control.tolist()


class JointPosition(Node):
    def __init__(self, shared_data, lock):
        super().__init__('joint_position')

        self.shared_data = shared_data
        self.lock = lock

        self.counter = 1
        self.current_positions = None
        self.sensor_signal = None
        self.initial_sensor_points = None  # Initialize the sensor points

        # Initialize variables
        self.motion_type = 1
        self.goal_positions = goal_positions
        self.velocity = 3.14
        self.acc_time = 0.0
        self.blend_percentage = 100
        self.fine_goal = False
        self.initial_sensor_points = []

        # Initialize shared data structure
        with self.lock:
            self.shared_data[100:106] = self.goal_positions

        # ROS2 Subscription setup to get joint states
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_position_callback, 10)

        # ROS2 Service client setup for setting positions
        self.client = self.create_client(SetPositions, "/set_positions")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SetPositions to become available...')
        self.send_positions(self.goal_positions)

        # New ROS2 Service client setup for MyService
        self.my_service_client = self.create_client(SetEvent, "/set_event")
        while not self.my_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SetEvent to become available...')

    def spin_in_thread(self):
        """This function will spin the ROS node separately."""
        rclpy.spin(self)

    def joint_position_callback(self, msg):
        """Callback to process the joint state data from the subscribed topic."""
        self.current_positions = msg.position
        with self.lock:
            self.shared_data[100:106] = np.array(msg.position)  # Update joint positions

    def send_positions(self, positions):
        """Send multiple service requests for different robot positions."""
        req = SetPositions.Request()
        req.motion_type = self.motion_type
        req.positions = positions
        req.velocity = self.velocity
        req.acc_time = self.acc_time
        req.blend_percentage = self.blend_percentage
        req.fine_goal = self.fine_goal
        self.client.call_async(req)  # set the position to the service client

    def send_clear_request(self):
        """Send a clear request using the SetEvent service."""
        clear_request = SetEvent.Request()
        clear_request.func = SetEvent.Request.STOP
        clear_request.arg0 = 0
        clear_request.arg1 = 0
        self.my_service_client.call_async(clear_request)

    def make_initial_sensor_points(self):
        """ 10*10 matrix with col wise distribution """
        R = radius  # Radius
        # y = np.linspace(-0.045, 0.045, 10)  # Generates 10 points directly
        # angle = np.linspace(40, 160, 10)  # Adjusting to ensure a semicircle or your specific arc
        y = y_position
        angle = sensor_angle

        # Preallocate arrays for x and z coordinates
        x = np.empty(0)
        z = np.empty(0)

        # Generate x and z coordinates for each angle, repeating for the entire column (y-coordinates)
        for ang in angle:
            x = np.append(x, np.full(shape=(10,), fill_value=R * np.cos(np.deg2rad(ang))))
            z = np.append(z, np.full(shape=(10,), fill_value=R * np.sin(np.deg2rad(ang))))

        # Repeat y-coordinates for each angle to match the repeated x and z coordinates
        y = np.tile(y, 10)  # Tile y-coordinates to match the layout

        # Stack x, y, and z to form the adjusted 3D point cloud, ensuring column-wise distribution
        self.initial_sensor_points = np.column_stack((-x, -y, z))
        return self.initial_sensor_points


class UI(Node):
    def __init__(self, shared_data, lock):
        self.T01 = np.array([[cos(goal_positions[0]), -sin(goal_positions[0]), 0, 0],
                             [sin(goal_positions[0]), cos(goal_positions[0]), 0, 0],
                             [0, 0, 1, 0.1452],
                             [0, 0, 0, 1]])
        self.T12 = np.array([[sin(goal_positions[1]), cos(goal_positions[1]), 0, 0],
                             [0, 0, 1, 0],
                             [cos(goal_positions[1]), -sin(goal_positions[1]), 0, 0],
                             [0, 0, 0, 1]])
        self.T23 = np.array([[cos(goal_positions[2]), -sin(goal_positions[2]), 0, 0.429],
                             [sin(goal_positions[2]), cos(goal_positions[2]), 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
        self.T34 = np.array([[cos(np.pi / 2 + goal_positions[3]), -sin(np.pi / 2 + goal_positions[3]), 0, 0.4115],
                             [sin(np.pi / 2 + goal_positions[3]), cos(np.pi / 2 + goal_positions[3]), 0, 0],
                             [0, 0, 1, -0.1223],
                             [0, 0, 0, 1]])
        self.T45 = np.array([[cos(goal_positions[4]), -sin(goal_positions[4]), 0, 0],
                             [0, cos(np.pi / 2), -sin(np.pi / 2), -0.106],
                             [sin(goal_positions[4]), cos(goal_positions[4]), 0, 0],
                             [0, 0, 0, 1]])
        self.T56 = np.array([[cos(goal_positions[5]), -sin(goal_positions[5]), 0, 0],
                             [0, cos(np.pi / 2), -sin(np.pi / 2), -0.11315],
                             [sin(goal_positions[5]), cos(goal_positions[5]), 0, 0],
                             [0, 0, 0, 1]])
        self.T_skin_0 = np.array([[0, 0.70710678, -0.70710678, 0],
                                  [-1, 0, 0, 0],
                                  [0, 0.70710678, 0.70710678, 0],
                                  [0, 0, 1, 1]])
        self.T_skin_1 = np.array([[0, -1, 0, 0.2144],
                                  [1, 0, 0, -0.000041],
                                  [0, 0, 1, -0.11642],
                                  [0, 0, 0, 1]])
        self.T_skin_2 = np.array([[0, -1, 0, 0.214],
                                  [1, 0, 0, 0],
                                  [0, 0, 1, 0.0048],
                                  [0, 0, 0, 1]])

        self.shared_data = shared_data  # This is a numpy array wrapping the RawArray
        self.lock = lock  # This is the lock for synchronizing access to the shared_data

        self.goal_positions = goal_positions
        self.TCP = None
        self.models = []
        self.skins = []
        self.meshes_skin = []
        self.names_skin = []
        self.pc_touchless = []
        self.proximity = []
        self.sensor_signal = None
        self.update_positions = self.goal_positions
        self.counter1 = 0
        self.counter2 = 0
        self.colors = np.zeros((1, 3))
        self.mesh = None
        self.elbow_mesh = None
        self.arm_mesh = None
        self.previous_sensor_data = None
        self.original_positions = None

        self.init_polyscope()  # Initialization

    def init_polyscope(self):
        ps.init()
        self.end_effector_sensor_initial_points()
        self.load_models()
        ps.set_user_callback(self.update_ui)
        ps.set_max_fps(-1)
        ps.set_up_dir("z_up")
        ps.set_program_name("ROBOT SKIN")
        ps.show()  # This will block this process and show the UI

    def load_models(self):
        folder_path = '/home/ping2/ros2_ws/src/robot_control/robot_control/resources/robot'
        obj_files = sorted(glob.glob(os.path.join(folder_path, '*.obj')))
        for obj_file in obj_files:
            file_name = os.path.splitext(os.path.basename(obj_file))[0]
            print("Robot Arm: ", file_name)
            mesh = om.read_trimesh(obj_file, face_color=True)
            colors = mesh.face_colors()
            vertices = mesh.points()
            faces = mesh.face_vertex_indices()
            model = ps.register_surface_mesh(file_name, vertices, faces)
            model.add_color_quantity("material", colors[:, :3], defined_on='faces', enabled=True)
            self.models.append(model)

        skin_folder_path = '/home/ping2/ros2_ws/src/robot_control/robot_control/resources/skin'
        skin_files = sorted(glob.glob(os.path.join(skin_folder_path, '*.obj')))
        for skin_file in skin_files:
            file_name = os.path.splitext(os.path.basename(skin_file))[0]
            if file_name == 'knitting_mesh_raw':
                continue
            print("Robot Skin: ", file_name)
            mesh = om.read_trimesh(skin_file)
            vertices = mesh.points()
            faces = mesh.face_vertex_indices()
            model = ps.register_surface_mesh(file_name, vertices, faces, color=[1, 0, 0])
            self.skins.append(model)
            self.meshes_skin.append(mesh)
            self.names_skin.append(file_name)

        self.models[5].set_transform(self.T01)
        self.models[0].set_transform(self.T01 @ self.T12)
        self.models[1].set_transform(self.T01 @ self.T12 @ self.T23)
        self.models[3].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T34)
        self.models[4].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45)
        self.models[6].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45 @ self.T56)
        self.models[7].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45 @ self.T56)
        self.skins[0].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T_skin_0)
        self.skins[1].set_transform(self.T01 @ self.T12 @ self.T_skin_1)
        self.skins[2].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T_skin_2)
        self.skins[3].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T_skin_0)

        self.pc_touchless[0].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45)
        # self.pc_touchless[1].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45)
        self.pc_touchless[2].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45)
        # self.pc_touchless[3].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T_skin_0)
        # self.pc_touchless[4].set_transform(self.T01 @ self.T12 @ self.T_skin_1)

        self.TCP.set_transform(self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45)

    def end_effector_sensor_initial_points(self):
        """ Parameters for the initial position of the sensor"""
        R = radius
        # y = np.linspace(-0.0405, 0.0405, 10)
        # angle = np.linspace(40, 160, 10)  # Adjusting to ensure a semicircle or your specific arc
        y = y_position
        angle = sensor_angle

        x = R * np.cos(np.deg2rad(angle))
        z = R * np.sin(np.deg2rad(angle))
        points = []
        for i in range(10):  # For each column (each x and z)
            for j in range(10):  # For each row in that column (each y)
                # Append the current point, with -x to match your coordinate system
                points.append((-x[i], -y[j], z[i]))
        points = np.array(points)
        self.proximity.append(points)

        """ Extract the vertices and faces from the .obj file and transform the vertices to the new position"""
        vertices = []
        faces = []
        with open('/home/ping2/ros2_ws/src/lan_control/lan_control/resources/sensor_1.obj', 'r') as file:
            for line in file:
                parts = line.split()
                if line.startswith('v '):
                    # Add vertices
                    vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
                elif line.startswith('f '):
                    # Add faces, adjusting from 1-based to 0-based index
                    faces.append([int(idx.split('/')[0]) - 1 for idx in parts[1:]])
        vertices = np.array(vertices)
        faces = np.array(faces)

        print("END EFFECTOR -- Length of Vertices: ", len(vertices), "Length of Faces: ", len(faces), flush=True)

        """ Colour for sensors """
        # colors = np.array([0.678, 0.847, 0.902] * len(faces)).reshape(len(faces), 3)  # dim: 11x11

        """ Create the point cloud and the transformed sensor mesh"""
        self.mesh = ps.register_surface_mesh("End Effector Sensor Mesh", vertices, faces, smooth_shade=True)
        # self.mesh.add_color_quantity("face_colors", colors, defined_on='faces', enabled=True)

        end_effector_signal = ps.register_point_cloud("end_effector_signal", points, radius=0.0018, color=[0, 0, 0])
        end_effector_sensor_vertex = ps.register_point_cloud("end_effector_sensor_vertex", vertices, radius=0.002, color=[1, 0, 0])
        end_effector_signal.update_point_positions(points)
        end_effector_sensor_vertex.update_point_positions(vertices)
        self.mesh.update_vertex_positions(vertices)
        self.pc_touchless.append(end_effector_signal)
        self.pc_touchless.append(end_effector_sensor_vertex)
        self.pc_touchless.append(self.mesh)
        self.TCP = ps.register_point_cloud("TCP", np.array([[0, 0, 0]]), radius=0.005, point_render_mode='sphere', color=[0, 0, 1])

        self.original_positions = np.array(self.proximity[0])

    def update_ui(self):
        """Periodically called method to refresh the UI based on the shared data."""
        with self.lock:
            positions = np.copy(self.shared_data[100:106])
            sensor_data = np.copy(self.shared_data[:100])
        # print("Sensor Data: ", sensor_data, flush=True)
        self.update_mesh_colors(sensor_data)
        self.update_transformations(positions)
        self.update_point_positions(sensor_data)

    def update_mesh_colors(self, sensor_data):
        """Update mesh colors based on sensor data, with colors normalized between 0 and 1 for Polyscope."""
        if self.mesh:
            num_faces = len(sensor_data)
            if not hasattr(self, 'previous_sensor_data') or not np.array_equal(self.previous_sensor_data, sensor_data):
                # Calculate colors only if there is a change in sensor data
                # Normalize color intensity from 0 to 1
                red_channel = np.ones(num_faces, dtype=float)  # Full intensity for red
                green_blue_intensity = 1 - sensor_data / 255.0  # Normalize and invert sensor data for green and blue channels

                # Stack arrays to form RGB colors
                new_colors = np.vstack((red_channel, green_blue_intensity, green_blue_intensity)).T

                # Update the mesh with new colors
                self.mesh.add_color_quantity("face_colors", new_colors, defined_on='faces', enabled=True)

                # Cache the current sensor data for change detection in future updates
                self.previous_sensor_data = sensor_data.copy()

    def update_point_positions(self, sensor_data):
        # Normalize sensor data from 0 to 255 to a range of 0 to 0.05 meters (5 cm)
        z_displacements = 0.05 * (1 - sensor_data / 255.0)  # Linear mapping: 0 -> +5cm, 255 -> original position  # dim: 11x11

        for i in range (len(sensor_data)):
            if sensor_data[i] == 0:
                z_displacements[i] = -0.02
        # Create new positions array from original, modify only the z-coordinate
        new_positions = np.copy(self.original_positions)
        new_positions[:, 2] += z_displacements  # Update z-coordinates based on calculated displacements

        # Update the point cloud positions with the new calculated positions
        self.pc_touchless[0].update_point_positions(new_positions)

    def update_transformations(self, update_positions):
        self.T01 = np.array([[cos(update_positions[0]), -sin(update_positions[0]), 0, 0],
                             [sin(update_positions[0]), cos(update_positions[0]), 0, 0],
                             [0, 0, 1, 0.1452],
                             [0, 0, 0, 1]])
        self.T12 = np.array([[sin(update_positions[1]), cos(update_positions[1]), 0, 0],
                             [0, 0, 1, 0],
                             [cos(update_positions[1]), -sin(update_positions[1]), 0, 0],
                             [0, 0, 0, 1]])
        self.T23 = np.array([[cos(update_positions[2]), -sin(update_positions[2]), 0, 0.429],
                             [sin(update_positions[2]), cos(update_positions[2]), 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
        self.T34 = np.array([[cos(np.pi / 2 + update_positions[3]), -sin(np.pi / 2 + update_positions[3]), 0, 0.4115],
                             [sin(np.pi / 2 + update_positions[3]), cos(np.pi / 2 + update_positions[3]), 0, 0],
                             [0, 0, 1, -0.1223],
                             [0, 0, 0, 1]])
        self.T45 = np.array([[cos(update_positions[4]), -sin(update_positions[4]), 0, 0],
                             [0, cos(np.pi / 2), -sin(np.pi / 2), -0.106],
                             [sin(update_positions[4]), cos(update_positions[4]), 0, 0],
                             [0, 0, 0, 1]])
        self.T56 = np.array([[cos(update_positions[5]), -sin(update_positions[5]), 0, 0],
                             [0, cos(np.pi / 2), -sin(np.pi / 2), -0.11315],
                             [sin(update_positions[5]), cos(update_positions[5]), 0, 0],
                             [0, 0, 0, 1]])

        self.models[5].set_transform(self.T01)
        self.models[0].set_transform(self.T01 @ self.T12)
        self.models[1].set_transform(self.T01 @ self.T12 @ self.T23)
        self.models[3].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T34)
        self.models[4].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45)
        self.models[6].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45 @ self.T56)
        self.models[7].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45 @ self.T56)
        self.skins[0].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T_skin_0)
        self.skins[1].set_transform(self.T01 @ self.T12 @ self.T_skin_1)
        self.skins[2].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T_skin_2)
        self.skins[3].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T_skin_0)
        self.pc_touchless[0].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45)  # sensor1 point_cloud signal
        # self.pc_touchless[1].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45)  # sensor1 mesh vertex
        self.pc_touchless[2].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45)  # sensor1 mesh
        # self.pc_touchless[3].set_transform(self.T01 @ self.T12 @ self.T23 @ self.T_skin_0)  # sensor2 point_cloud
        # self.pc_touchless[4].set_transform(self.T01 @ self.T12 @ self.T_skin_1)  # arm mesh

        self.TCP.set_transform(self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45)

def run_game(shared_data, lock):
    np_shared_data = np.frombuffer(shared_data, dtype=float)  # Wrap the RawArray with a NumPy array for easier manipulation
    commander = sensor_serial_api_old.ArduinoCommander(serial_port, baud_rate)
    game = MyGame(np_shared_data, lock, SCREEN_WIDTH, SCREEN_HEIGHT, "Touchless Sensor UI", commander)

    arcade.run()


def run_ros_node(shared_data, lock):
    rclpy.init()
    np_shared_data = np.frombuffer(shared_data, dtype=float)  # Wrap the RawArray with a NumPy array for easier manipulation

    joint_position_class = JointPosition(shared_data, lock)  # Adjust the constructor to accept shared data and lock
    joint_position_class.make_initial_sensor_points()
    apf_class = ArtificialPotentialField(joint_position_class, shared_data, lock)
    apf_class.transformation_matrix(joint_position_class.goal_positions)

    counter = 0

    try:
        ros_thread = threading.Thread(target=rclpy.spin, args=(joint_position_class,))
        ros_thread.start()
        while rclpy.ok():
            print("Counter in main: ", counter, flush=True)
            counter += 1

            time.sleep(0.4)

            with lock:
                waypoint = np_shared_data[100:106]  # Assume first 6 elements are current positions

            # print(waypoint, flush=True)
            apf_class.transformation_matrix(waypoint)

            fatt: np.ndarray = np.zeros((6, 3))
            frep = apf_class.calculate_repulsive_force()

            if apf_class.diff_norm(joint_position_class.current_positions, goal_positions) < 0.01 and apf_class.matrix_row_norm(frep) < 0.001:
                # print("Done")
                continue

            qdot = []

            if apf_class.diff_norm(joint_position_class.current_positions, goal_positions) > 0.01 and apf_class.matrix_row_norm(frep) < 0.001:
                # print("Entered 11111111", flush=True)

                for i in range(6):
                    diff = joint_position_class.current_positions[i] - goal_positions[i]
                    f = 0
                    if diff < d:
                        f = -zeta * diff  # dim: 1x1
                    else:
                        f = -zeta * d
                    qdot.append(f)

                if apf_class.norm(qdot) > 0.02:
                    # print("Entered 22222222", flush=True)
                    qdot = apf_class.normalize(qdot, 0.02)

                with lock:
                    for j in range(len(qdot)):
                        waypoint[j] += qdot[j]

                joint_position_class.send_positions(waypoint)

            else:
                qdot = apf_class.control_update(frep, fatt)
                qdot = apf_class.normalize(qdot, 0.30)

                with lock:
                    for j in range(len(qdot)):
                        waypoint[j] += qdot[j]

                joint_position_class.send_positions(waypoint)

            if apf_class.norm(qdot) < 0.00001:
                continue  # Continue the loop or perform another action

    except KeyboardInterrupt:
        print("Shutting down...")

    finally:
        joint_position_class.destroy_node()
        rclpy.shutdown()


def run_ui(shared_data, lock):
    np_shared_data = np.frombuffer(shared_data, dtype=float)  # Wrap the RawArray with a NumPy array for easier manipulation
    ui = UI(np_shared_data, lock)  # Initialize UI with shared memory array and lock
    ui.run()


def create_shared_array(size, ctype='d'):  # 'd' for double
    return RawArray(ctype, size)


def main():
    shared_data = create_shared_array(106)  # Adjust the size according to your data structure
    lock = Lock()  # Create a lock

    print("You disabled the index 3, 13, 23, 33, 43, 53, 63, 73, 83, 93", flush=True)

    ros_process = Process(target=run_game, args=(shared_data, lock))
    ros_process2 = Process(target=run_ros_node, args=(shared_data, lock))
    ui_process = Process(target=run_ui, args=(shared_data, lock))

    ros_process.start()
    ros_process2.start()
    ui_process.start()

    ros_process.join()
    ros_process2.join()
    ui_process.join()


if __name__ == '__main__':
    main()


