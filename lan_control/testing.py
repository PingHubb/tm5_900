import time
import arcade
import os
import csv
import rclpy
import threading
import numpy as np
import glob
from math import sin, cos
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from lan_control import sensor_serial_api

# Serial Communication Setup
baud_rate = 9600

class SensorDataPublisher(Node):
    def __init__(self):
        super().__init__('sensor_data_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, 'heatmap_data_topic', 10)
        self.publisher_2 = self.create_publisher(Float32MultiArray, 'heatmap_data_topic_2', 10)

    def publish_data(self, data):
        msg = Float32MultiArray()
        msg.data = data
        self.publisher.publish(msg)
        # self.get_logger().info(f'Publishing: {data}, {len(data)}')

    def publish_data_2(self, data):
        msg = Float32MultiArray()
        msg.data = data
        self.publisher_2.publish(msg)


def get_serial_ports():
    # Get a list of all serial ports
    ports = glob.glob('/dev/ttyACM*')
    # Sort the list to ensure consistent ordering
    ports.sort()
    return ports


def main():
    rclpy.init()
    serial_ports = get_serial_ports()

    commander = sensor_serial_api.ArduinoCommander(serial_ports, baud_rate)
    ros_publisher = SensorDataPublisher()

    # Start ROS2 node in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_publisher,), daemon=True)
    spin_thread.start()

    while True:

        count_time = time.time()
        data = commander.send_command('readRaw')
        print("data: ", data)
        print(f"Time taken: {time.time() - count_time}")


if __name__ == '__main__':
    main()