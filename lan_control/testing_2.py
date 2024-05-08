import polyscope as ps
from multiprocessing import Process, Queue, RawArray, Lock
import glob
import os
import openmesh as om
import rclpy
from rclpy.node import Node
from rclpy import logging
import numpy as np
import math
import threading
import time
from math import cos, sin, radians
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String
from tm_msgs.srv import SetEvent, SetPositions
from sensor_msgs.msg import JointState


class Sensor(Node):
    def __init__(self, shared_data, lock):
        super().__init__('sensor_signal')
        self.shared_data = shared_data
        self.lock = lock
        self.counter = 1
        self.sensor_signal = None
        self.initial_sensor_points = []

        with self.lock:
            self.shared_data[0:100] = np.zeros(100)

        # ROS2 Subscription setup to get sensor signal
        self.subscription_sensor = self.create_subscription(Float32MultiArray, '/heatmap_data_topic', self.sensor_callback, 10)

    def spin_in_thread(self):
        """This function will spin the ROS node separately."""
        rclpy.spin(self)

    def sensor_callback(self, signal):
        """Callback to process the sensor data from the subscribed topic."""
        self.sensor_signal = signal.data
        with self.lock:
            self.shared_data[0:100] = np.array(signal.data)  # Update sensor signals


class UI(Node):
    def __init__(self, shared_data, lock):

        self.shared_data = shared_data  # This is a numpy array wrapping the RawArray
        self.lock = lock  # This is the lock for synchronizing access to the shared_data

        self.models = []
        self.skins = []
        self.meshes_skin = []
        self.names_skin = []
        self.pc_touchless = []
        self.proximity = []
        self.sensor_signal = None
        self.counter1 = 0
        self.counter2 = 0
        self.colors = np.zeros((1, 3))
        self.mesh = None
        self.previous_sensor_data = None

        self.init_polyscope()  # Initialization

    def init_polyscope(self):
        ps.init()
        self.load_models()
        ps.set_user_callback(self.update_ui)
        ps.set_max_fps(-1)
        ps.set_up_dir("z_up")
        ps.set_program_name("SKIN")
        ps.show()  # This will block this process and show the UI

    def group_vertices_by_faces(self):
        faces = []
        current_face = []
        current_color = None

        with open('/home/ping2/Downloads/conference/1.obj', 'r') as file:
            vertex_index = 0
            for line in file:
                parts = line.strip().split()
                if parts[0] == 'v':
                    x, y, z = map(float, parts[1:4])
                    r, g, b = map(float, parts[4:7])
                    normalized_color = (r / 255.0, g / 255.0, b / 255.0)

                    # Start a new face if the color changes
                    if current_color is None or current_color == normalized_color:
                        current_face.append(vertex_index)
                    else:
                        faces.append(current_face)
                        current_face = [vertex_index]

                    current_color = normalized_color
                    vertex_index += 1

            # Add the last face if any vertices remain
            if current_face:
                faces.append(current_face)

        return faces, vertex_index

    def load_models(self):
        vertices = []
        faces = []
        with open('/home/ping2/Downloads/conference/tri.obj', 'r') as file:
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

        self.mesh = ps.register_surface_mesh("End Effector Sensor Mesh", vertices, faces, smooth_shade=True)
        self.mesh.update_vertex_positions(vertices)
        self.pc_touchless.append(self.mesh)

        vertices = []
        colors = []
        with open('/home/ping2/Downloads/conference/1.obj', 'r') as file:
            for line in file:
                parts = line.split()
                if line.startswith('v '):
                    # Add vertices
                    vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
                    colors.append([float(parts[4]), float(parts[5]), float(parts[6])])  # assuming color values are normalized
        vertices = np.array(vertices)
        colors = np.array(colors)

        sensor_pc = ps.register_point_cloud("sensor pc", vertices, radius=0.002)
        sensor_pc.add_color_quantity("color", colors, enabled=True)
        sensor_pc.update_point_positions(vertices)
        self.pc_touchless.append(sensor_pc)

        faces_by_color, total_vertices = self.group_vertices_by_faces()
        print("len(faces_by_color): ", len(faces_by_color))

    def update_ui(self):
        """Periodically called method to refresh the UI based on the shared data."""
        with self.lock:
            sensor_data = np.copy(self.shared_data[:100])


def run_ros_node(shared_data, lock):
    rclpy.init()
    np_shared_data = np.frombuffer(shared_data, dtype=float)  # Wrap the RawArray with a NumPy array for easier manipulation

    sensor_class = Sensor(shared_data, lock)  # Adjust the constructor to accept shared data and lock

    counter = 0

    try:
        ros_thread = threading.Thread(target=rclpy.spin, args=(sensor_class,))
        ros_thread.start()
        while rclpy.ok():
            # print("Counter in main: ", counter, flush=True)
            counter += 1
            # print(np_shared_data[:100])

    except KeyboardInterrupt:
        print("Shutting down...")

    finally:
        sensor_class.destroy_node()
        rclpy.shutdown()


def run_ui(shared_data, lock):
    np_shared_data = np.frombuffer(shared_data, dtype=float)  # Wrap the RawArray with a NumPy array for easier manipulation
    ui = UI(np_shared_data, lock)  # Initialize UI with shared memory array and lock
    ui.run()


def create_shared_array(size, ctype='d'):  # 'd' for double
    return RawArray(ctype, size)


def main():
    shared_data = create_shared_array(100)  # Adjust the size according to your data structure
    lock = Lock()  # Create a lock

    ros_process = Process(target=run_ros_node, args=(shared_data, lock))
    ui_process = Process(target=run_ui, args=(shared_data, lock))

    ros_process.start()
    ui_process.start()

    ros_process.join()
    ui_process.join()


if __name__ == '__main__':
    main()



