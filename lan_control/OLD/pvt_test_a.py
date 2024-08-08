import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import scrolledtext
import threading
import time
from lan_control.OLD import sensor_serial_api_old
import os


# Serial Communication Setup
serial_port = '/dev/ttyACM0'
baud_rate = 9600

class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, 'positions_topic', 10)

    def publish_positions_with_duration(self, positions, duration, velocity):
        msg = Float64MultiArray()
        msg.data = positions + [duration] + velocity
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def perform_readings(commander, step, file, text_widget):
    file.write(f"--- Step {step} ---\n")
    for _ in range(400):
        data = commander.read_raw()
        data_str = ', '.join(map(str, data)) if isinstance(data, list) else str(data)
        text_widget.insert(tk.END, f"Step {step}: {data_str}\n")
        file.write(data_str + "\n")
        time.sleep(0.005)

def print_raw_data(commander, text_widget):
    data = commander.read_raw()
    data_str = ', '.join(map(str, data)) if isinstance(data, list) else str(data)
    text_widget.insert(tk.END, f"Raw Data: {data_str}\n")

def start_process(text_widget):
    rclpy.init()
    publisher_node = PositionPublisher()
    commander = sensor_serial_api_old.ArduinoCommander(serial_port, baud_rate)

    positions_c = [-455, -568, 385, 180.0, 0.0, 0.0]
    positions_b = [-471, -568, 385, 180.0, 0.0, 0.0]
    positions_a = [-487, -568, 385, 180.0, 0.0, 0.0]
    velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    duration = 0.1

    a = positions_a

    with open('/home/ping2/ros2_ws/src/lan_control/sensor_readings.txt', 'w') as file:
        # Initialization ---------------------------------------------------------
        publisher_node.publish_positions_with_duration(a, duration, velocity)
        time.sleep(1)
        text_widget.insert(tk.END, f"Done Initialization\n")
        for i in range(1):
            publisher_node.publish_positions_with_duration(a, duration, velocity)
            time.sleep(1)
        text_widget.insert(tk.END, f"Done Second Initialization\n")
        # Initialization ---------------------------------------------------------

        for i in range(1):
            a[2] -= 100
            text_widget.insert(tk.END, f"1111111111111\n")
            publisher_node.publish_positions_with_duration(a, duration, velocity)
            time.sleep(2)
            perform_readings(commander, i + 1, file, text_widget)

        for i in range(1):
            text_widget.insert(tk.END, f"22222222222222\n")
            publisher_node.publish_positions_with_duration(a, duration, velocity)
            time.sleep(2)
            perform_readings(commander, i + 1, file, text_widget)

        for i in range(1):
            a[2] += 100
            text_widget.insert(tk.END, f"333333333333333\n")
            publisher_node.publish_positions_with_duration(a, duration, velocity)
            time.sleep(2)
            perform_readings(commander, i + 1, file, text_widget)

    rclpy.shutdown()


def main():
    root = tk.Tk()
    root.title("ROS Control Panel")

    text_widget = scrolledtext.ScrolledText(root, height=50, width=100)
    text_widget.pack(pady=20)

    start_button = tk.Button(root, text="Start", command=lambda: threading.Thread(target=start_process, args=(text_widget,)).start())
    start_button.pack(pady=10)

    commander = sensor_serial_api_old.ArduinoCommander(serial_port, baud_rate)  # Initialize the commander

    print_raw_button = tk.Button(root, text="Print Raw", command=lambda: print_raw_data(commander, text_widget))
    print_raw_button.pack(pady=10)

    root.mainloop()

if __name__ == '__main__':
    main()
