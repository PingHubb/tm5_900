import arcade
import os
import csv
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from lan_control.API import sensor_serial_api
from lan_control.OLD import distance_calculation_mujoco

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

first_set_of_data = 500

""" For Demo """
useless_row = 11  # Count from bottom to top, start from 0   11
useless_col = -1  # Count from left to right, start from 0   -1
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
        self.cell_value_history = []
        self.matrix_height = 0
        self.matrix_width = 0
        self.show_graph = False  # Add this line
        self.y_axis_center_value = None
        self.smoothed_curve_data = []
        self.initial_data_collected = False
        self.threshold_max = None
        self.threshold_min = None
        self.initial_threshold_calculated = False
        self.threshold_diff = None
        self.average_of_first_data_smoothed = None  # Initialize the attribute
        self.initial_threshold_max = None
        self.initial_threshold_min = None
        self.filtered_data = []
        self.new_filtered_data = []
        self.new_smoothed_curve_data = []
        self.graph_background_color = arcade.color.WHITE_SMOKE  # Default graph background color
        self.threshold_triggered = False  # To track if the threshold condition is met
        self.current_threshold_max = None
        self.current_threshold_min = None
        self.moving_average_window = 10  # Window size for moving average

        self.processed_data = {}  # Stores processed data for all cells
        self.process_all = False



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
        # Calculate IQR for outlier detection :)
        q1, q3 = np.percentile(self.cell_value_history[-100:], [25, 75])
        iqr = q3 - q1
        lower_bound = q1 - 1.5 * iqr
        upper_bound = q3 + 1.5 * iqr

        self.new_filtered_data = [x for x in self.cell_value_history[-100:] if lower_bound <= x <= upper_bound]
        self.new_smoothed_curve_data = self.calculate_moving_average(self.new_filtered_data)

        new_smoothed_data = []
        new_window_size = 10

        for i in range(len(self.new_smoothed_curve_data)):
            window = self.new_smoothed_curve_data[max(0, i - new_window_size + 1):i + 1]
            new_smoothed_data.append(np.mean(window))
        self.average_of_first_data_smoothed = np.mean(new_smoothed_data)

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

    def draw_heatmap(self):
        global useless_row, useless_col

        # Min and Max values for the heatmap
        min_value = 0
        max_value = 2000

        self.matrix_height = self.channel_check[0] - 1  # Height of the matrix
        self.matrix_width = self.channel_check[1]  # Width of the matrix
        square_size = 60  # Size of each square in the heatmap
        start_x = SCREEN_WIDTH * 1.4 / 2 - (square_size * self.matrix_width) / 2
        start_y = SCREEN_HEIGHT / 2 - (square_size * self.matrix_height) * 0.9 / 2

        for row in range(self.matrix_height):
            if row == useless_row:  # Skip this row if it's the useless row
                continue
            for col in range(self.matrix_width):
                if col == useless_col:  # Skip this column if it's the useless column
                    continue

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
                self.heatmap_data = distance_calculation_mujoco.process_data(self.calibration_data, self.raw_data)[:remove_last_how_many_value]
                # self.heatmap_data = distance_calculation_mujoco.process_data(self.calibration_data, self.raw_data)
                # print("len(self.heatmap_data): ", len(self.heatmap_data))
                self.heatmap_data[-10:] = [-777] * 10  # <---------------------

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


    def on_mouse_press(self, x, y, button, modifiers):
        if button == arcade.MOUSE_BUTTON_LEFT:
            for button in self.button_list:
                button.check_click(x, y)

    def on_mouse_motion(self, x, y, dx, dy):
        for button in self.button_list:
            button.update_hover(x, y)


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


