import arcade
import threading
import time
import rclpy
import numpy as np
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String
from tm_msgs.srv import SetEvent, SetPositions
from sensor_msgs.msg import JointState

# Constants
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
SCREEN_TITLE = "Robot Controller"

class JointPosition(Node):
    def spin_in_thread(self):
        """This function will spin the ROS node separately."""
        rclpy.spin(self)

    def __init__(self):
        super().__init__('joint_position')
        self.current_position = None
        self.new_callback_received = False  # New attribute

        # Initialize variables
        self.motion_type = 1
        self.target_position = [-0.2, 0.0, 1.58, 0.0, 1.58, 0.0]
        self.old_target_position = [-0.2, 0.0, 1.58, 0.0, 1.58, 0.0]
        self.velocity = 3.14
        self.acc_time = 0.0  # MUST BE 0 to become FAST
        self.blend_percentage = 100  # MUST BE 100 to become SMOOTH
        self.fine_goal = False  # MUST BE False to become SMOOTH

        # ROS2 Subscription setup to get joint states
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_position_callback, 10)

        # ROS2 Service client setup for setting positions
        self.client = self.create_client(SetPositions, "/set_positions")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SetPositions to become available...')
        self.send_positions(self.target_position)

    def joint_position_callback(self, msg):
        """Callback to process the joint state data from the subscribed topic."""
        # self.get_logger().info(f"Positions: {msg.position}")
        self.current_position = msg.position
        self.new_callback_received = True  # Set the flag to True when new data is received
        # print(msg.speed)


    def send_positions(self, positions):
        """Send multiple service requests for different robot positions."""
        req = SetPositions.Request()
        req.motion_type = self.motion_type
        req.positions = positions
        req.velocity = self.velocity
        req.acc_time = self.acc_time
        req.blend_percentage = self.blend_percentage
        req.fine_goal = self.fine_goal
        self.client.call_async(req)  # <---- set the position to the service client

class MyGame(arcade.Window):
    def __init__(self, width, height, title, joint_node):
        super().__init__(width, height, title)
        self.joint_node = joint_node
        self.angle_increment = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]  # Change this value as needed
        self.error_threshold_0 = 0.09
        self.error_threshold_1 = 0.09
        self.error_threshold_2 = 0.09
        self.error_threshold_3 = 0.09
        self.error_threshold_4 = 0.09
        self.error_threshold_5 = 0.09
        self.current_error = 0.0  # Initialize with a large number
        self.counter = 0
        self.run_test = False  # New flag for test function

    def on_draw(self):
        arcade.start_render()

    def on_key_press(self, key, modifiers):

        # Adjust joint angles
        if key in [arcade.key.NUM_1, arcade.key.NUM_3, arcade.key.NUM_5, arcade.key.NUM_7, arcade.key.NUM_9]:
            joint_index = {arcade.key.NUM_1: 0, arcade.key.NUM_3: 1, arcade.key.NUM_5: 2, arcade.key.NUM_7: 3, arcade.key.NUM_9: 4}[key]
            self.angle_increment[joint_index] += 0.05
            print("angle_increment: ", self.angle_increment)

        elif key in [arcade.key.NUM_2, arcade.key.NUM_4, arcade.key.NUM_6, arcade.key.NUM_8, arcade.key.NUM_0]:
            joint_index = {arcade.key.NUM_2: 0, arcade.key.NUM_4: 1, arcade.key.NUM_6: 2, arcade.key.NUM_8: 3, arcade.key.NUM_0: 4}[key]
            self.angle_increment[joint_index] -= 0.05
            print("angle_increment: ", self.angle_increment)

        # Return to starting point when 'o' is pressed
        elif key == arcade.key.O:
            self.run_test = False
            self.joint_node.target_position = [-0.2, 0.0, 1.58, 0.0, 1.58, 0.0]
            self.joint_node.old_target_position = [-0.2, 0.0, 1.58, 0.0, 1.58, 0.0]
            self.angle_increment = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.joint_node.send_positions(self.joint_node.target_position)

        elif key == arcade.key.X:
            print("X key pressed")
            self.run_test = True
            increment_array = np.array(self.angle_increment)  # Convert the list to a numpy array
            self.joint_node.target_position = self.joint_node.current_position + increment_array
            self.error_threshold_0 = abs((self.joint_node.target_position[0] - self.joint_node.current_position[0]) / 2)
            self.error_threshold_1 = abs((self.joint_node.target_position[1] - self.joint_node.current_position[1]) / 2)
            self.error_threshold_2 = abs((self.joint_node.target_position[2] - self.joint_node.current_position[2]) / 2)
            self.error_threshold_3 = abs((self.joint_node.target_position[3] - self.joint_node.current_position[3]) / 2)
            self.error_threshold_4 = abs((self.joint_node.target_position[4] - self.joint_node.current_position[4]) / 2)
            self.error_threshold_5 = abs((self.joint_node.target_position[5] - self.joint_node.current_position[5]) / 2)

        elif key == arcade.key.P:
            threading.Thread(target=self.robot_run).start()

    def robot_run(self):
        print("Test function started")
        self.counter = 0
        while True:
            if self.run_test:
                if self.joint_node.new_callback_received:
                    self.joint_node.new_callback_received = False
                    self.current_error = np.array(self.joint_node.old_target_position) - np.array(self.joint_node.current_position)
                    print("Current Error: ", self.current_error)

                    error_check = True
                    for i in range(6):
                        threshold = getattr(self, f'error_threshold_{i}')
                        if threshold > 0 and abs(self.current_error[i]) >= threshold:
                            error_check = False
                            break

                    if error_check:
                        print("Error: ", self.current_error)
                        self.run_test = False  # Reset the flag
                        self.joint_node.send_positions(self.joint_node.target_position)
                        self.joint_node.old_target_position = self.joint_node.target_position
                        print("Test function executed")


def main(args=None):
    rclpy.init(args=args)
    joint_node = JointPosition()

    # print("Current position: ", joint_node.current_position)

    # Start ROS2 node in a separate thread
    node_thread = threading.Thread(target=lambda: rclpy.spin(joint_node))
    node_thread.start()

    # Run Arcade window in the main thread
    window = MyGame(400, 400, "Arcade Window", joint_node)
    arcade.run()

    joint_node.destroy_node()
    rclpy.shutdown()
    node_thread.join()


if __name__ == '__main__':
    main()
