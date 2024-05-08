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

goal_joint_states = [0.8, 0.0, 1.58, 0.0, 1.58, 0.0]

one_time = True
baby = 0

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

    # def joint_velocities(self, end_effector_velocity):
    #     J = self.jacobian()
    #     J_inv = np.linalg.pinv(J)  # Moore-Penrose pseudo-inverse
    #     return J_inv @ end_effector_velocity  # This is the joint velocities vector, which tells you how fast each joint should move to achieve the desired end-effector velocity.
    #
    # def calculate_joint_torques(self, end_effector_forces): # <--- This is the function you need to implement
    #     J_v = self.jacobian()[0:3, :] # dim: 3x6
    #     return J_v.T @ end_effector_forces
    #
    # def end_effector_velocity(self, joint_velocities):
    #     J = self.jacobian()
    #     return J @ joint_velocities
    #
    # # Optionally, if you want to compute forces at the end-effector given joint torques
    # def calculate_end_effector_forces(self, joint_torques):
    #     J = self.jacobian()
    #     return np.linalg.pinv(J.T) @ joint_torques

    def get_jacobian_by_index(self, jac, idx):
        # Create a zero matrix with the same dimensions as the Eigen::MatrixXd::Zero(3, 6)
        self.id_jac = np.zeros((3, 6))

        # Copy elements up to the given index
        for i in range(idx + 1):
            self.id_jac[0, i] = jac[0, i]
            self.id_jac[1, i] = jac[1, i]
            self.id_jac[2, i] = jac[2, i]

        return self.id_jac

    # output example:
    # The Jacobian Matrix(jac):
    # [[0.68715595 0.48039264 0.9387282  0.52083424 0.67858569 0.99061846]
    #  [0.83398381 0.67415119 0.33698704 0.73400275 0.99722533 0.92860093]
    #  [0.89468805 0.90116297 0.97302047 0.66762661 0.61733461 0.48589859]]
    #
    # The extracted Jacobian by index(id_jac):
    # [[0.68715595 0.48039264 0.9387282  0.         0.         0.]
    #  [0.83398381 0.67415119 0.33698704 0.         0.         0.]
    #  [0.89468805 0.90116297 0.97302047 0.         0.         0.]]

    # Example usage:
    # jacobian_handler = JacobianHandler()
    # jac = np.random.rand(3, 6)  # Assuming jac is a 3x6 matrix
    # idx = 2 # We want to use the first three columns (0 to idx)
    # id_jac = jacobian_handler.get_jacobian_by_index(jac, idx) = [[0.68715595 0.48039264 0.9387282  0.         0.         0.]

    def diff_norm(self, current_positions, goal_joint_states):
        # Convert lists to numpy arrays if they aren't already
        self.current_positions_variable = np.asarray(current_positions)
        self.goal_joint_states_variable = np.asarray(goal_joint_states)

        # Calculate the L2 norm (Euclidean distance)
        self.l2_sum = np.sum((self.current_positions_variable - self.goal_joint_states_variable) ** 2)
        return np.sqrt(self.l2_sum)

    # Example usage:
    # q1 = [1.0, 2.0, 3.0]
    # q2 = [4.0, 5.0, 6.0]
    # print(diff_norm(q1, q2))  =  5.19615242271

    # def vec_interpolation(self, current_positions, goal_joint_states, size):
    #     self.res = []
    #     self.current_positions_vec_interpolation = np.array(current_positions)
    #     self.goal_joint_states_vec_interpolation = np.array(goal_joint_states)
    #
    #     # Calculate the interpolated vectors
    #     for i in range(size):
    #         inter_q = []
    #         for j in range(len(self.current_positions_vec_interpolation)):
    #             inter_q.append(self.current_positions_vec_interpolation[j] + (self.goal_joint_states_vec_interpolation[j] - self.current_positions_vec_interpolation[j]) / (size - 1) * i)
    #         self.res.append(inter_q)
    #     return self.res

        # Example usage:
        # q1 = [1, 2, 3, 4, 5, 6]
        # q2 = [6, 5, 4, 3, 2, 1]
        # size = 5
        # interpolated_vectors = vec_interpolation(q1, q2, size)
        # output:
        # [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        # [2.25, 2.75, 3.25, 3.75, 4.25, 4.75]
        # [3.5, 3.5, 3.5, 3.5, 3.5, 3.5]
        # [4.75, 4.25, 3.75, 3.25, 2.75, 2.25]
        # [6.0, 5.0, 4.0, 3.0, 2.0, 1.0]

    def norm(self, q):
        l2_sum = sum(x ** 2 for x in q)
        return math.sqrt(l2_sum)

    # Example usage:
    # q = np.array([1.0, 2.0, 3.0])
    # print(norm(q))  = 3.74165738677

    def normalize(self, q, scale=1.0):
        normal = self.norm(q)
        if normal == 0:
            return q  # Return the original vector if its norm is 0 to avoid division by zero
        return [x / normal * scale for x in q]

    # Example usage:
    # q = np.array([1.0, 2.0, 3.0])
    # normalized_q = normalize(q)
    # print(normalized_q) = [0.26726124 0.53452248 0.80178373]

    # With scaling:
    # scaled_normalized_q = normalize(q, scale=2.0)
    # print(scaled_normalized_q) = [0.53452248 1.06904496 1.60356744]

    def matrix_row_norm(self, mat):
        # # Compute the norm of each row and sum them
        # if mat.ndim == 1:
        #     self.row_norm = np.linalg.norm(mat)
        # elif mat.ndim == 2:
        #     self.row_norm = np.sum(np.linalg.norm(mat, axis=1))
        # return self.row_norm

        self.total_norm = 0
        for row in mat:
            row_norm = sum([element ** 2 for element in row]) ** 0.5
            self.total_norm += row_norm
        return self.total_norm

    # Example usage:
    # mat = np.array([[1, 2, 3], [4, 5, 6]])
    # print(matrix_row_norm(mat))
    # Output: Matrix row norm: 14.2828568570857

    # def vec_absolute_max(self, q):
    #     return max(q, key=abs)

    # Example usage:
    # q = [-10.5, 3.25, -2.75, 9.0, 10.4]
    # print("Maximum absolute value:", vec_absolute_max(q))
    # Output: Maximum absolute value: 10.5

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
        self.location_radians = [0.8, 0.0, 1.58, 0.0, 1.58, 0.0]
        self.velocity = 3.14
        self.acc_time = 0.0
        self.blend_percentage = 100
        self.fine_goal = False
        self.initial_sensor_points = []

        self.pt_touchless = []
        self.pc_touchless = []
        self.distance_touchless = []
        self.normals_touchless = []

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

        R = 0.0906 / 2
        y = [-0.045 + j * 0.01 for i in range(11) for j in range(10)]

        angle = [20.0 + 14.0 * i for i in range(11)]

        x = [R * np.cos(np.radians(ang)) for ang in angle for _ in range(10)]
        z = [R * np.sin(np.radians(ang)) for ang in angle for _ in range(10)]

        self.initial_sensor_points = []
        for xi, yi, zi in zip(x, y, z):
            point = np.array([xi, yi, zi])
            self.initial_sensor_points.append(point) # dim: 110x3

        # print("Initial sensor points: ", self.initial_sensor_points, flush=True)
        return self.initial_sensor_points # dim: 110x3

    def calculate_repulsive_force(self, forward_kinematic_class):

        self.Frep = np.zeros((6, 3))  # Create a matrix of zeros
        sensor_info = self.sensor_signal  # Accessing the sensor_signal attribute
        link5_transform = forward_kinematic_class.T05
        self.delta = np.zeros(3)  # Assuming delta is a 3D vector

        for i in range(110):
            if sensor_info[i] < 0.0:
                continue
            if abs(sensor_info[i] - 1) > 253.9:
                local_point = self.initial_sensor_points[i] # dim: 3x1
                local_translate = np.eye(4) # dim: 4x4
                local_translate[:3, 3] = local_point # dim: 3x1
                local_normal = np.array([local_point[0], 0, local_point[2]]) # dim: 3x1

                list_version = local_point.tolist()
                # print(list_version, flush=True)

                # if i in {0, 1, 2, 3, 10, 11, 12, 13, 14, 20, 21, 22, 23, 30, 31, 32, 33}:
                #     # Move to right bottom
                #     local_normal = [-1, 0, 0]
                # elif i in {6, 7,  8,  9, 16, 17, 18, 19, 26, 27, 28, 29, 36, 37, 38, 39}:
                #     # Move to left bottom
                #     local_normal = [1, 1, -1]
                # elif i in {70, 71, 72,73,80,81,82,83,90,91,92,93,100,101,102,103}:
                #     # Move to right top
                #     local_normal = [-1, -1, 1]
                # elif i in {76,77,78,79,86,87,88,89,96,97,98,99,106,107,108,109}:
                #     # Move to left top
                #     local_normal = [1, 0, 0]
                # # elif i in {4,5,14,15,24,25,34,35}:
                # #     # Move to the downward
                # #     local_normal = [0, 0.5, -0.5]
                # elif i in {4,5,14,15,24,25,34,35, 44, 45, 54,55,64,65,74,75, 84,85,94,95,104,105}:
                #     # Move to backward
                #     # local_normal = [0.03, 0.03, -0.05]
                #     # local_normal = [0, -0.5, 0.5]  # <-- this is upward
                #     local_normal = [1, 0, 0]
                # elif i in {40,41,42,43,50,51,52,53,60,61,62,63}:
                #     # Move to the right
                #     local_normal = [-1.1, -0.5, 0.5]
                # elif i in [46,47,48,49,56,57,58,59,66,67,68,69]:
                #     # Move to the left
                #     local_normal = [1.4, 0.5, -0.5]
                # else:
                #     continue

                # local_normal = forward_kinematic_class.normalize(local_normal) # dim: 3x1
                # local_normal = np.array(local_normal)  # Convert local_normal to a NumPy array
                #
                # local_normal *= sensor_info[i] * 0.01 # dim: 3x1
                # local_offset = np.eye(4) # dim: 4x4
                # local_offset[:3, 3] = local_normal # dim: 3x1 # local_offset = distance from sensor to surface
                #
                # mapped_point = (link5_transform @ local_translate @ local_offset)[:3, 3] # dim: 3x1
                # surface_point = (link5_transform @ local_translate)[:3, 3] # dim: 3x1
                #
                # vec = surface_point - mapped_point
                # self.delta += vec

                local_point = self.initial_sensor_points[i]
                local_normal = np.array([local_point[0], 0, local_point[2]])
                local_normal = forward_kinematic_class.normalize(local_normal)

                local_normal = np.array(local_normal)  # Ensure local_normal is a NumPy array
                sensor_info = np.array(sensor_info)  # Convert sensor_info to a NumPy array

                local_normal *= sensor_info[i] * 0.01
                local_translate = np.eye(4)
                local_translate[:3, 3] = local_point
                local_offset = np.eye(4)
                local_offset[:3, 3] = local_normal

                mapped_point = (link5_transform @ local_translate @ local_offset)[:3, 3]
                surface_point = (link5_transform @ local_translate)[:3, 3]
                vec = surface_point - mapped_point
                self.delta += vec

        if forward_kinematic_class.norm(self.delta) < 0.0001:
            pass

        else:
            self.delta = forward_kinematic_class.normalize(self.delta)
            dist = 0.01  # this seems to be a fixed small distance value
            f = eta * (1 / dist - 1 / rho) * (1 / (dist**2)) * np.array(self.delta)
            self.Frep[4, :] = f  # Assuming that the repulsive force is applied to the 5th row (index 4)

        # print(f"Frep: ", self.Frep, flush=True)
        return self.Frep

    # def calculate_attractive_force(self, joint_position_class):
    #
    #     # Initialize matrices to store the current and goal positions of each link
    #     cur_link_positions = joint_position_class.latest_positions
    #     goal_link_positions = [0.0, 0.0, 1.58, 0.0, 1.58, 0.0]
    #
    #     self.Fatt = np.zeros((6, 3))  # Create a matrix of zeros
    #
    #     self.Fatt = np.zeros((len(cur_link_positions), 3))
    #
    #     for i in range(len(cur_link_positions)):
    #         diff = np.array(cur_link_positions[i]) - np.array(goal_link_positions[i])
    #         error = np.linalg.norm(diff)
    #         if error < d:
    #             f = -zeta * diff
    #         else:
    #             f = -d * zeta * diff / error
    #         self.Fatt[i, :] = f
    #
    #     return self.Fatt

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
    global one_time, baby
    rclpy.init(args=args)

    joint_position_class = JointPosition()
    joint_position_class.make_initial_sensor_points()
    forward_kinematic_class = ForwardKinematic()
    forward_kinematic_class.transformation_matrix(joint_position_class.location_radians)

    try:
        ros_thread = threading.Thread(target=rclpy.spin, args=(joint_position_class,))
        ros_thread.start()

        while rclpy.ok():

            if one_time is True:
                time.sleep(1)
                one_time = False

            # print(baby, flush=True)
            # baby += 1

            waypoint = joint_position_class.latest_positions
            forward_kinematic_class.transformation_matrix(waypoint)

            # fatt = joint_position_class.calculate_attractive_force(joint_position_class)
            fatt = np.zeros((6, 3))  # Create a matrix of zeros
            frep = joint_position_class.calculate_repulsive_force(forward_kinematic_class)

            # print(f"Frep: ", frep, flush=True)
            # print(f"Fatt: ", fatt, flush=True)
            # diff_norm_value = forward_kinematic_class.diff_norm(waypoint, goal_joint_states)
            # print(f"diff_norm: {diff_norm_value:.5f}", flush=True)
            # print(f"matrix_row_norm(frep): ", forward_kinematic_class.matrix_row_norm(frep), flush=True)

            if forward_kinematic_class.diff_norm(joint_position_class.latest_positions, goal_joint_states) < 0.01 and forward_kinematic_class.matrix_row_norm(frep) < 0.001:
                # print("Done")
                continue

            qdot = []

            if forward_kinematic_class.diff_norm(joint_position_class.latest_positions, goal_joint_states) > 0.01 and forward_kinematic_class.matrix_row_norm(frep) < 0.001:
                print("Entered 11111111", flush=True)

                for i in range(6):
                    diff = joint_position_class.latest_positions[i] - goal_joint_states[i]
                    f = 0
                    if diff < d:
                        f = -zeta * diff # dim: 1x1
                    else:
                        f = -zeta * d
                    qdot.append(f)
                    # print(f"i = ", i, flush=True)

                if forward_kinematic_class.norm(qdot) > 0.02:
                    print("Entered 22222222", flush=True)
                    qdot = forward_kinematic_class.normalize(qdot, 0.02)

                # Update waypoint
                for j in range(len(qdot)):
                    waypoint[j] += qdot[j]
                    # print(f"j = ", j, flush=True)

                joint_position_class.send_positions(waypoint)
                # print(f"qdot: ", qdot, flush=True)
                # print(f"waypoint: ", waypoint, flush=True)
                # joint_position_class.send_clear_request()
                time.sleep(0.5)

            else:
                print("Entered 3333333", flush=True)
                qdot = joint_position_class.control_update(forward_kinematic_class, joint_position_class, frep, fatt)
                qdot = forward_kinematic_class.normalize(qdot, 0.30)

                for j in range(len(qdot)):
                    waypoint[j] += qdot[j]

                joint_position_class.send_positions(waypoint)
                # print(f"qdot: ", qdot, flush=True)
                # print(f"waypoint: ", waypoint, flush=True)
                time.sleep(0.5)

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
