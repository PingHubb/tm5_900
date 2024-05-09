import polyscope as ps
from multiprocessing import Process, Queue, RawArray, Lock
import glob
import os
import openmesh as om
import rclpy
from rclpy.node import Node
from rclpy import logging
from scipy.spatial import Delaunay
from collections import defaultdict
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
        self.pc_touchless = []
        self.proximity = []
        self.grouped_vertices = None
        self.sensor_signal = None
        self.colors = np.zeros((1, 3))
        self.mesh = None

        self.init_polyscope()  # Initialization

    def init_polyscope(self):
        ps.init()
        self.load_models()
        ps.set_user_callback(self.update_ui)
        ps.set_max_fps(-1)
        ps.set_up_dir("z_up")
        ps.set_program_name("SKIN")
        ps.show()  # This will block this process and show the UI


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
        # colors = np.ones((len(vertices), 3)) * [1, 0, 0]
        # print("length of colors: ", len(colors))
        # colors[:200] = [0, 1, 0]
        # self.mesh.add_color_quantity("mesh_color", colors, enabled=True)
        self.mesh.update_vertex_positions(vertices)
        self.pc_touchless.append(self.mesh)

        # Path to the text file with group indices
        obj_file_path = '/home/ping2/Downloads/conference/1.obj'
        text_file_path = '/home/ping2/Downloads/conference/signal.txt'

        # Load the group indices from the text file
        group_indices = []
        with open(text_file_path, 'r') as file:
            for line in file:
                group_indices.append(int(line.strip()))

        # Load vertices from the OBJ file
        vertices = []
        with open(obj_file_path, 'r') as file:
            for line in file:
                if line.startswith('v '):
                    parts = line.split()
                    vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])

        self.grouped_vertices = defaultdict(list)

        for index, vertex in enumerate(vertices):
            group = group_indices[index]
            if group != -1:  # Exclude vertices with group index -1
                self.grouped_vertices[group].append(vertex)

        # for group_id, verts in self.grouped_vertices.items():
        #     verts_np = np.array(verts)
        #     pc = ps.register_point_cloud(f"Group {group_id}", verts_np, radius=0.002)
        #     self.models.append(pc)  # Store the reference to the point cloud

    def update_ui(self):
        """Periodically called method to refresh the UI based on the shared data."""
        with self.lock:
            sensor_data = np.copy(self.shared_data[:100])
            # self.colour_point_cloud(sensor_data)

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



