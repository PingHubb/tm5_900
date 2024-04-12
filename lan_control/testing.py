from typing import Any
import rclpy
from numpy import ndarray, dtype, floating, float_
from numpy._typing import _64Bit
from rclpy.node import Node
from rclpy import logging
import numpy as np
import math
import threading
import time
from math import cos, sin
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String
from tm_msgs.srv import SetEvent, SetPositions
from sensor_msgs.msg import JointState

zeta = 1.0
d = 0.01
rho = 0.1
eta = 0.0001

goal_positions = [1.0, 0.0, 1.57, 0.0, 1.57, 0.0]
# goal_positions = [math.radians(-180), math.radians(45), math.radians(45), math.radians(0), math.radians(90), math.radians(0)]
# goal_positions = [-4.28372736e-03, -4.88533739e-01, 1.57943555e+00, -1.09090181e+00 + math.pi / 2, 1.56651260e+00, -1.95721289e-12]

class ArtificialPotentialField:
    def __init__(self, joint_position_instance):
        self.joint_position_instance = joint_position_instance
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
        # print(f"Jacobian: ", self.J, flush=True)

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
            # print(f"linear_velocity_component: ", linear_velocity_component, flush=True)

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
        sensor_info = self.joint_position_instance.sensor_signal  # Accessing the sensor_signal attribute
        link5_transform = self.T05
        self.delta = np.zeros(3)  # Assuming delta is a 3D vector

        for i in range(100):
            if sensor_info[i] > 50:
                local_point = self.joint_position_instance.initial_sensor_points[i]  # dim: 3x1
                local_translate = np.eye(4)  # np.eye[4] creates a 4x4 identity matrix
                local_translate[:3, 3] = local_point  # select the first 3 rows and the 4th column and assign local_point to it

                local_normal = self.normalize(np.array([local_point[0], local_point[1], local_point[2]]))  # Normalize the local_normal vector
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
        current_joint_states = self.joint_position_instance.current_positions

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
    def __init__(self):
        super().__init__('joint_position')

        self.counter = 1
        self.current_positions = None
        self.sensor_signal = None
        self.initial_sensor_points = None  # Initialize the sensor points

        # Initialize variables
        self.motion_type = 1
        self.goal_positions = [1.0, 0.0, 1.57, 0.0, 1.57, 0.0]
        self.velocity = 3.14
        self.acc_time = 0.0
        self.blend_percentage = 100
        self.fine_goal = False
        self.initial_sensor_points = []

        # ROS2 Subscription setup to get joint states
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_position_callback, 10)

        # ROS2 Subscription setup to get sensor signal
        self.subscription_sensor = self.create_subscription(Float32MultiArray, '/heatmap_data_topic', self.sensor_callback, 10)

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

    def sensor_callback(self, signal):
        """Callback to process the sensor data from the subscribed topic."""
        self.sensor_signal = signal.data
        self.counter += 1

    def send_positions(self, positions):
        """Send multiple service requests for different robot positions."""
        req = SetPositions.Request()
        req.motion_type = self.motion_type
        req.positions = positions
        req.velocity = self.velocity
        req.acc_time = self.acc_time
        req.blend_percentage = self.blend_percentage
        req.fine_goal = self.fine_goal
        self.client.call_async(req) # <---- set the position to the service client

    def send_clear_request(self):
        """Send a clear request using the SetEvent service."""
        clear_request = SetEvent.Request()
        clear_request.func = SetEvent.Request.STOP
        clear_request.arg0 = 0
        clear_request.arg1 = 0
        self.my_service_client.call_async(clear_request)

    def make_initial_sensor_points(self):
        # """ 11*10 matrix with row wise distribution"""
        # R = 0.0906 / 2  # Radius
        # # Adjust y-coordinates to have 11 steps, evenly spaced between -0.045 and 0.045
        # # y = np.linspace(-0.045, 0.045, 11)  # Generates 11 points directly
        # y = np.linspace(-0.045, 0.045, 11)  # Generates 11 points directly
        # y = np.repeat(y, 10)  # Now, repeat these 11 y-coordinates 10 times each
        #
        # # Generate angles. Since we need 10 points around the part of the circle, adjust the range accordingly
        # # angle = np.linspace(20, 160, 10)  # Generates 10 points
        # angle = np.linspace(50, 190, 10)  # Generates 10 points
        #
        # # Calculate x and z coordinates based on the generated angles
        # x = R * np.cos(np.deg2rad(angle))
        # x = np.tile(x, 11)  # Tile x-coordinates to match the new distribution of y
        # z = R * np.sin(np.deg2rad(angle))
        # z = np.tile(z, 11)  # Tile and invert z-coordinates
        #
        # # Stack x, y, and z to form the adjusted 3D point cloud
        # self.initial_sensor_points = np.column_stack((-x, -y, z))
        # print(f"Initial Sensor Points: ", self.initial_sensor_points, flush=True)
        #
        # return self.initial_sensor_points

        """ 10*10 matrix with col wise distribution """
        R = 0.0906 / 2  # Radius

        # Adjust y-coordinates to have 10 steps, evenly spaced between -0.045 and 0.045
        y = np.linspace(-0.045, 0.045, 10)  # Generates 10 points directly
        # Generate angles for a 10-point circular layout, adjusting the range
        angle = np.linspace(40, 140, 10)  # Adjusting to ensure a semicircle or your specific arc

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
        print("Initial Sensor Points:", self.initial_sensor_points)
        print("Initial Sensor Points Shape:", self.initial_sensor_points.shape)

        return self.initial_sensor_points


def main(args=None):
    rclpy.init(args=args)

    joint_position_class = JointPosition()
    joint_position_class.make_initial_sensor_points()
    apf_class = ArtificialPotentialField(joint_position_class)
    apf_class.transformation_matrix(joint_position_class.goal_positions)

    try:
        ros_thread = threading.Thread(target=rclpy.spin, args=(joint_position_class,))
        ros_thread.start()

        while rclpy.ok():

            time.sleep(0.4)
            waypoint = joint_position_class.current_positions
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

                # Update waypoint
                for j in range(len(qdot)):
                    waypoint[j] += qdot[j]

                joint_position_class.send_positions(waypoint)

            else:
                qdot = apf_class.control_update(frep, fatt)
                qdot = apf_class.normalize(qdot, 0.30)

                # Update waypoint
                for j in range(len(qdot)):
                    waypoint[j] += qdot[j]

                joint_position_class.send_positions(waypoint)

            if apf_class.norm(qdot) < 0.00001:
                continue  # Continue the loop or perform another action

    except KeyboardInterrupt:
        pass

    finally:
        joint_position_class.destroy_node()
        rclpy.shutdown()
        ros_thread.join()  # Wait for the ros_thread to finish


if __name__ == '__main__':
    main()

