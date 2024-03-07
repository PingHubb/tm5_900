import rclpy
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

goal_joint_states = [0.0, 0.0, 1.58, 0.0, 1.58, 0.0]

class ForwardKinematic:
    def __init__(self):
        for attr in ['T01', 'T12', 'T23', 'T34', 'T45', 'T56', 'T02', 'T03', 'T04', 'T05', 'T06', 'J']:
            setattr(self, attr, None)

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
        self.T06 = self.T05 @ self.T56

        # Now, let's also compute the Jacobian
        self.J = self.jacobian()

    def jacobian(self):  # Based on the geometric approach
        J = np.zeros((6, 6))
        end_effector_pos = self.T06[:3, 3]  # Extracting the position of the end-effector

        transforms = [np.eye(4), self.T01, self.T02, self.T03, self.T04, self.T05]

        for i in range(6):
            joint_pos = transforms[i][:3, 3]
            z_axis = transforms[i][:3, 2]  # z-axis after i-th transformation

            # Linear velocity part (top 3 rows of the Jacobian)
            J[:3, i] = np.cross(z_axis, end_effector_pos - joint_pos)

            # Angular velocity part (bottom 3 rows of the Jacobian)
            J[3:, i] = z_axis

        return J

    def get_jacobian_by_index(self, jac, idx):
        # Create a zero matrix with the same dimensions as the Eigen::MatrixXd::Zero(3, 6)
        self.id_jac = np.zeros((3, 6))

        # Copy elements up to the given index
        for i in range(idx + 1):
            self.id_jac[0, i] = jac[0, i]
            self.id_jac[1, i] = jac[1, i]
            self.id_jac[2, i] = jac[2, i]

        return self.id_jac

    def diff_norm(self, current_positions, goal_joint_states):
        # Convert lists to numpy arrays if they aren't already
        self.current_positions_variable = np.asarray(current_positions)
        self.goal_joint_states_variable = np.asarray(goal_joint_states)

        # Calculate the L2 norm (Euclidean distance)
        self.l2_sum = np.sum((self.current_positions_variable - self.goal_joint_states_variable) ** 2)
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


class JointPosition(Node):
    def spin_in_thread(self):
        """This function will spin the ROS node separately."""
        rclpy.spin(self)

    def __init__(self):
        super().__init__('joint_position')

        self.counter = 1
        self.latest_positions = None
        self.sensor_signal = None
        self.initial_sensor_points = None  # Initialize the sensor points

        # Initialize variables
        self.motion_type = 1
        self.location_radians = [0.0, 0.0, 1.58, 0.0, 1.58, 0.0]
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
        self.send_positions(self.location_radians)

        # New ROS2 Service client setup for MyService
        self.my_service_client = self.create_client(SetEvent, "/set_event")
        while not self.my_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SetEvent to become available...')

    def joint_position_callback(self, msg):
        """Callback to process the joint state data from the subscribed topic."""
        # self.get_logger().info(f"Positions: {msg.position}")
        self.latest_positions = msg.position
        # ForwardKinematic().transformation_matrix(msg.position)

    def sensor_callback(self, signal):
        """Callback to process the sensor data from the subscribed topic."""
        self.sensor_signal = signal.data
        # print("List No.", self.counter)
        # print(', '.join(map(str, self.sensor_signal)), flush=True)
        self.counter += 1
        # return self.sensor_signal

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
        # *************** Below is 11*10 = 110 points *************** #

        R = 0.0906 / 2  # Radius, assuming this is consistent with Skin_Thread logic

        # Sensor positions
        y = [-0.045 + j * 0.01 for j in range(10) for _ in range(11)]

        # Assuming the same angular logic as in Skin_Thread
        angle = [1 * np.pi * 7 / 6 - j * np.pi * 7 / 66 - np.pi / 6 for j in range(10)]

        x = [R * np.cos(ang) for ang in angle for _ in range(11)]
        z = [R * np.sin(ang) for ang in angle for _ in range(11)]

        self.initial_sensor_points = np.array([x, y, z]).T

        return self.initial_sensor_points

    def calculate_repulsive_force(self, forward_kinematic_class):

        self.Frep = np.zeros((6, 3))  # Create a matrix of zeros
        sensor_info = self.sensor_signal  # Accessing the sensor_signal attribute
        link5_transform = forward_kinematic_class.T05
        self.delta = np.zeros(3)  # Assuming delta is a 3D vector

        for i in range(110):
            if abs(sensor_info[i] - 1000) < 0.0001:
                local_point = self.initial_sensor_points[i] # dim: 3x1
                print(f"local_point: ", local_point, flush=True)
                local_translate = np.eye(4) # dim: 4x4
                local_translate[:3, 3] = local_point # dim: 3x1
                local_normal = np.array([local_point[0], 0, local_point[2]]) # dim: 3x1

                if np.linalg.norm(local_normal) > 0:  # To avoid division by zero
                    local_normal /= np.linalg.norm(local_normal)
                local_normal = forward_kinematic_class.normalize(local_normal) # dim: 3x1
                local_normal = np.array(local_normal)  # Convert local_normal to a NumPy array

                local_normal *= sensor_info[i] * 0.01 # dim: 3x1
                local_offset = np.eye(4) # dim: 4x4
                local_offset[:3, 3] = local_normal # dim: 3x1 # local_offset = distance from sensor to surface

                mapped_point = (link5_transform @ local_translate @ local_offset)[:3, 3] # dim: 3x1
                surface_point = (link5_transform @ local_translate)[:3, 3] # dim: 3x1

                vec = surface_point - mapped_point
                self.delta += vec

            # Calculate the repulsive force based on the delta
        if forward_kinematic_class.norm(self.delta) < 0.0001:
            pass

        else:
            self.delta = forward_kinematic_class.normalize(self.delta)
            dist = 0.01  # this seems to be a fixed small distance value
            f = eta * (1 / dist - 1 / rho) * (1 / (dist**2)) * np.array(self.delta)
            self.Frep[4, :] = f  # Assuming that the repulsive force is applied to the 5th row (index 4)

        return self.Frep

    def control_update(self, forward_kinematic_class, joint_position_class, Frep, Fatt):
        F = Fatt + Frep
        jac = forward_kinematic_class.jacobian()
        current_joint_states = joint_position_class.latest_positions

        control = np.zeros(6) # dim: 6x1

        for i in range(5, 6):
            jac_i = forward_kinematic_class.get_jacobian_by_index(jac, i)
            v = jac_i.T @ Fatt[i, :].T
            control += v

        for i in range(6):
            jac_i = forward_kinematic_class.get_jacobian_by_index(jac, i)
            v = jac_i.T @ Frep[i, :].T
            control += v

        if forward_kinematic_class.norm(control) < 0.05 and forward_kinematic_class.diff_norm(current_joint_states, goal_joint_states) > 1.0:
            # Log local minimum correction
            for j in range(6):
                control[j] += (goal_joint_states[j] - current_joint_states[j]) * 0.01

        return control.tolist()

def main(args=None):
    rclpy.init(args=args)

    joint_position_class = JointPosition()
    joint_position_class.make_initial_sensor_points()
    forward_kinematic_class = ForwardKinematic()
    forward_kinematic_class.transformation_matrix(joint_position_class.location_radians)

    try:
        ros_thread = threading.Thread(target=rclpy.spin, args=(joint_position_class,))
        ros_thread.start()

        while rclpy.ok():

            time.sleep(0.5)
            waypoint = joint_position_class.latest_positions
            forward_kinematic_class.transformation_matrix(waypoint)

            fatt = np.zeros((6, 3))  # Create a matrix of zeros
            frep = joint_position_class.calculate_repulsive_force(forward_kinematic_class)

            if forward_kinematic_class.diff_norm(joint_position_class.latest_positions, goal_joint_states) < 0.01 and forward_kinematic_class.matrix_row_norm(frep) < 0.001:
                print("Done")
                continue

            qdot = []

            if forward_kinematic_class.diff_norm(joint_position_class.latest_positions, goal_joint_states) > 0.01 and forward_kinematic_class.matrix_row_norm(frep) < 0.001:
                # print("Entered 11111111", flush=True)

                for i in range(6):
                    diff = joint_position_class.latest_positions[i] - goal_joint_states[i]
                    f = 0
                    if diff < d:
                        f = -zeta * diff # dim: 1x1
                    else:
                        f = -zeta * d
                    qdot.append(f)

                if forward_kinematic_class.norm(qdot) > 0.02:
                    # print("Entered 22222222", flush=True)
                    qdot = forward_kinematic_class.normalize(qdot, 0.02)

                # Update waypoint
                for j in range(len(qdot)):
                    waypoint[j] += qdot[j]

                joint_position_class.send_positions(waypoint)

            else:
                qdot = joint_position_class.control_update(forward_kinematic_class, joint_position_class, frep, fatt)
                qdot = forward_kinematic_class.normalize(qdot, 0.30)

                # Update waypoint
                for j in range(len(qdot)):
                    waypoint[j] += qdot[j]

                joint_position_class.send_positions(waypoint)

            if forward_kinematic_class.norm(qdot) < 0.00001:
                continue  # Continue the loop or perform another action


    except KeyboardInterrupt:
        pass

    finally:
        joint_position_class.destroy_node()
        rclpy.shutdown()
        ros_thread.join()  # Wait for the ros_thread to finish

if __name__ == '__main__':
    main()
