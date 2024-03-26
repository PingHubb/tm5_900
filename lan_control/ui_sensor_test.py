import time
import arcade
import os
import csv
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from lan_control import sensor_serial_api, distance_calculation, distance_calculation_test

# Serial Communication Setup
serial_port = '/dev/ttyACM0'
baud_rate = 9600

# Constants for the UI
SCREEN_WIDTH = 1200  # 1100
SCREEN_HEIGHT = 900  # 700
BUTTON_WIDTH = 250
BUTTON_HEIGHT = 50
BUTTON_X_POSITIONS = SCREEN_WIDTH / 5  # Adjust based on your layout
BUTTON_Y_POSITIONS = [800, 600, 530, 460, 390, 320, 250, 150, 50]  # Adjust based on your layout and number of buttons
NEON_COLOR = (58, 255, 217)  # A neon-like color
HOVER_COLOR = (255, 235, 59)  # Color when hovered

number_of_data = 100

""" For Demo """
# useless_row = 11  # Count from bottom to top, start from 0
# useless_col = -1  # Count from left to right, start from 0
remove_last_how_many_value = -10  # Remove the last 10 values from the heatmap data

class SensorDataPublisher(Node):
    def __init__(self):
        super().__init__('sensor_data_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, 'heatmap_data_topic', 10)

    def publish_data(self, data):
        msg = Float32MultiArray()
        msg.data = data
        self.publisher.publish(msg)
        # self.get_logger().info(f'Publishing: {data}, {len(data)}')


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
    def __init__(self, width, height, title, commander, ros_publisher):
        super().__init__(width, height, title)
        background_path = os.path.expanduser("~/Downloads/nature.jpg")  # Expand the path to the full directory
        self.commander = commander
        self.ros_publisher = ros_publisher
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
        self.initial_data_collected = False
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
        self.current_smoothed_value = {}

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
        print(f"Calibration Data is equal to: {self.calibration_data}")

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

    # def draw_heatmap(self):
    #     global useless_row, useless_col
    #
    #     # Min and Max values for the heatmap
    #     min_value = 0
    #     max_value = 5000
    #
    #     self.matrix_height = self.channel_check[0] - 1   # Height of the matrix
    #     self.matrix_width = self.channel_check[1]  # Width of the matrix
    #     square_size = 60  # Size of each square in the heatmap
    #     start_x = SCREEN_WIDTH * 1.4 / 2 - (square_size * self.matrix_width) / 2
    #     start_y = SCREEN_HEIGHT / 2 - (square_size * self.matrix_height) * 0.9 / 2
    #
    #     for row in range(self.matrix_height):
    #         # if row == useless_row:  # Skip this row if it's the useless row
    #         #     continue
    #         for col in range(self.matrix_width):
    #             # if col == useless_col:  # Skip this column if it's the useless column
    #             #     continue
    #
    #             x = start_x + col * square_size
    #             y = start_y + row * square_size
    #             index = row * self.matrix_width + col
    #             value = self.heatmap_data[index] if index < len(self.heatmap_data) else 0
    #
    #             # Check if this is the selected cell
    #             if self.selected_cell == (row, col):
    #                 color = arcade.color.BLUE  # Change color to blue for the selected cell
    #             elif min_value <= value <= max_value:
    #                 red_intensity = int(255 * (value - min_value) / (max_value - min_value))
    #                 color = (255, 255 - red_intensity, 255 - red_intensity)  # From white to red
    #             else:
    #                 color = arcade.color.LIGHT_GRAY
    #
    #             arcade.draw_rectangle_filled(x, y, square_size, square_size, color)
    #             text_color = arcade.color.BLACK if value <= 1250 else arcade.color.WHITE  # Improved visibility
    #             arcade.draw_text(str(value), x, y, text_color, 12, width=square_size, align="center", anchor_x="center",
    #                              anchor_y="center")

    def draw_heatmap(self):
        global useless_row, useless_col

        # Min and Max values for the heatmap
        min_value = 0
        max_value = 5000

        self.matrix_height = self.channel_check[0] - 1   # Height of the matrix
        self.matrix_width = self.channel_check[1]  # Width of the matrix
        square_size = 60  # Size of each square in the heatmap
        start_x = SCREEN_WIDTH * 1.4 / 2 - (square_size * self.matrix_width) / 2
        start_y = SCREEN_HEIGHT / 2 - (square_size * self.matrix_height) * 0.9 / 2

        for row in range(self.matrix_height):
            # if row == useless_row:  # Skip this row if it's the useless row
            #     continue
            for col in range(self.matrix_width):
                # if col == useless_col:  # Skip this column if it's the useless column
                #     continue

                x = start_x + col * square_size
                y = start_y + row * square_size
                index = row * self.matrix_width + col
                value = self.heatmap_data[index] if index < len(self.heatmap_data) else 0

                # Check if this is the selected cell
                if self.selected_cell == (row, col):
                    color = arcade.color.BLUE  # Change color to blue for the selected cell
                elif min_value <= value <= max_value:
                    red_intensity = int(255 * (value - min_value) / (max_value - min_value))
                    color = (255, 255 - red_intensity, 255 - red_intensity)  # From white to red
                elif self.threshold_triggered.get((row, col), False) is True:
                    color = arcade.color.GREEN

                else:
                    color = arcade.color.LIGHT_GRAY

                arcade.draw_rectangle_filled(x, y, square_size, square_size, color)
                text_color = arcade.color.BLACK if value <= 1250 else arcade.color.WHITE  # Improved visibility
                arcade.draw_text(str(value), x, y, text_color, 12, width=square_size, align="center", anchor_x="center",
                                 anchor_y="center")

    def record_heatmap_data(self):
        self.is_recording = not self.is_recording  # Toggle recording state
        if self.is_recording:
            print("Recording started.")
        else:
            print("Recording stopped.")

    def record_to_csv(self):
        filename = "heatmap_data.csv"
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
        if self.automation_active:
            self.counter += 1
            self.perform_read_raw()
            if self.raw_data and self.calibration_data:
                # Update the heatmap data
                # self.heatmap_data = distance_calculation_test.process_data(self.calibration_data, self.raw_data)[:remove_last_how_many_value]
                # self.heatmap_data = distance_calculation_mujoco.process_data(self.calibration_data, self.raw_data)
                self.heatmap_data = self.raw_data[:remove_last_how_many_value]  # <---------- Test for raw data

                """ For switching col to row major"""
                # Create an empty matrix with the desired dimensions
                row_major_matrix = [[None for _ in range(10)] for _ in range(11)]
                # Populate the matrix in row-major order
                for index, value in enumerate(self.heatmap_data):
                    row = index % 11
                    col = index // 11
                    row_major_matrix[row][col] = value
                flattened_data = [item for sublist in row_major_matrix for item in sublist]
                """ For switching col to row major"""

                self.ros_publisher.publish_data(flattened_data)

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
                print("index: ", index, "value: ", self.heatmap_data[index])

        if self.process_all:
            self.process_all_cells()

    def process_all_cells(self):
        """Preprocess and store data for all cells."""
        if not self.processed_data:  # Initialize if empty
            for row in range(self.matrix_height):
                for col in range(self.matrix_width):
                    self.processed_data[(row, col)] = []
            print(f"Processed data initialized with {self.matrix_height} x {self.matrix_width} cells.")
            print(f"Length of processed data: {len(self.processed_data)}")

        if self.processed_data:
            # a = time.time()  # Start the timer
            for row in range(self.matrix_height):
                for col in range(self.matrix_width):
                    cell_index = row * self.matrix_width + col  # matrix_width is 10, because 10 columns
                    if cell_index < len(self.heatmap_data):  # Check if the index is within the data range, which is 110
                        self.process_cell_data(row, col, self.heatmap_data[cell_index])  # Process the data for each cell repeatedly
                    else:
                        print(f"Data for cell ({row}, {col}) is out of range and cannot be processed.")
            # print("Time taken to process all cells: ", time.time() - a)  # Print the time taken to process all cells

    def process_cell_data(self, row, col, data):  # Only 1 cell of data is entering here each time
        """Append data for a single cell identified by row and col."""
        self.processed_data[(row, col)].append(data)

        if len(self.processed_data[(row, col)]) > number_of_data:
            self.processed_data[(row, col)].pop(0)
        if len(self.processed_data[(row, col)]) == number_of_data:
            self.set_thresholds(row, col, self.processed_data[(row, col)])

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
            self.threshold_max[(row, col)] = max(self.filtered_data[(row, col)])
            self.threshold_min[(row, col)] = min(self.filtered_data[(row, col)])
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

    def check_heatmap_click(self, x, y):
        square_size = 60
        start_x = SCREEN_WIDTH * 1.4 / 2 - (square_size * self.matrix_width) / 2
        start_y = SCREEN_HEIGHT / 2 - (square_size * self.matrix_height) * 0.9 / 2

        for row in range(self.matrix_height):
            for col in range(self.matrix_width):
                cell_x = start_x + col * square_size - 30
                cell_y = start_y + row * square_size - 30
                # Check if the click is within the bounds of the cell
                if (cell_x <= x <= cell_x + square_size and
                    cell_y <= y <= cell_y + square_size):
                    self.selected_cell = (row, col)

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
            self.draw_threshold_line(self.threshold_max[(row, col)], arcade.color.GREEN, y_min, y_max, y_scale, graph_top, graph_height, graph_left, graph_width)
            self.draw_threshold_line(self.threshold_min[(row, col)], arcade.color.RED, y_min, y_max, y_scale, graph_top, graph_height, graph_left, graph_width)

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

    def check_threshold_condition(self):
        any_cell_triggered = False  # Track if any cell has triggered the threshold

        for row in range(self.matrix_height):
            for col in range(self.matrix_width):
                cell_key = (row, col)

                # Assume the cell has not triggered the threshold initially
                self.threshold_triggered[cell_key] = False

                if cell_key in self.smoothed_curve_data and len(self.smoothed_curve_data[cell_key]) >= 5:
                    last_data = self.smoothed_curve_data[cell_key][-5:]
                    above_threshold = all(d > self.threshold_max[cell_key] for d in last_data)
                    below_threshold = all(d < self.threshold_min[cell_key] for d in last_data)

                    if above_threshold or below_threshold:
                        self.threshold_triggered[cell_key] = True
                        any_cell_triggered = True  # At least one cell has triggered the threshold

        # Update the global background color based on whether any cell has triggered
        if any_cell_triggered:
            self.graph_background_color = arcade.color.BLACK
        else:
            self.graph_background_color = arcade.color.WHITE_SMOKE


def main():
    rclpy.init()
    commander = sensor_serial_api.ArduinoCommander(serial_port, baud_rate)
    ros_publisher = SensorDataPublisher()
    game = MyGame(SCREEN_WIDTH, SCREEN_HEIGHT, "Touchless Sensor UI", commander, ros_publisher)

    # Start ROS2 node in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_publisher,), daemon=True)
    spin_thread.start()

    # Start the Arcade event loop
    arcade.run()

    # Once the Arcade window is closed, shutdown ROS
    rclpy.shutdown()


if __name__ == '__main__':
    main()


