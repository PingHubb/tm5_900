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

from tm_msgs.msg import FeedbackState
import ikpy.chain
from scipy.spatial.transform import Rotation as R
import mujoco as mj
import mujoco.viewer

"""
Mujoco Model Setup
"""
m = mj.MjModel.from_xml_path(
    '/home/ping2/Desktop/Mujoco_Adimttance_Control/Github_Version/Mujoco_Admittance_Control_TM5-900/TM5-900/TM5-900_description/TM5/scene.xml')
m.opt.gravity[0] = 0.0  # x轴重力
m.opt.gravity[1] = 0.0  # y轴重力
m.opt.gravity[2] = 0.0  # z轴重力
d = mj.MjData(m)
m.opt.timestep = 0.01

"""
Ikpy Setup
"""
# End Effector-joint6 to joint3
TM5_Joint_7 = ikpy.chain.Chain.from_urdf_file(
    "/home/ping2/Desktop/Mujoco_Adimttance_Control/Github_Version/Mujoco_Admittance_Control_TM5-900/TM5-900/script/Different_Joint_Test/TM5_7.urdf")
TM5_Joint_6 = ikpy.chain.Chain.from_urdf_file(
    "/home/ping2/Desktop/Mujoco_Adimttance_Control/Github_Version/Mujoco_Admittance_Control_TM5-900/TM5-900/script/Different_Joint_Test/TM5_6.urdf")
TM5_Joint_5 = ikpy.chain.Chain.from_urdf_file(
    "/home/ping2/Desktop/Mujoco_Adimttance_Control/Github_Version/Mujoco_Admittance_Control_TM5-900/TM5-900/script/Different_Joint_Test/TM5_5.urdf")
TM5_Joint_4 = ikpy.chain.Chain.from_urdf_file(
    "/home/ping2/Desktop/Mujoco_Adimttance_Control/Github_Version/Mujoco_Admittance_Control_TM5-900/TM5-900/script/Different_Joint_Test/TM5_4.urdf")

"""
Admittance Algorithm
"""
# Mass Matrix
M_param = 1e1
M = np.diag([M_param, M_param, M_param, M_param, M_param, M_param])

# Damping Ratio
d_param = 200  # 200
d_param_rot = 200  # 200
D = np.diag([d_param, d_param, d_param, d_param_rot, d_param_rot, d_param_rot])
D = d_param

# K
k_param = 500  # 500
k_param_rot = 20  # 20
K = np.diag([k_param, k_param, k_param, k_param_rot, k_param_rot, k_param_rot])
K = k_param

# Velocity and Acceleration Limit
arm_max_acc_ = 100.0  # Maximum acceleration
arm_max_vel_ = 100.0  # Maximum velocity

alpha = np.array(
    [-4.28372736e-03, -4.88533739e-01, 1.57943555e+00, -1.09090181e+00 + math.pi / 2, 1.56651260e+00, -1.95721289e-12])
wrench_external = np.zeros(6)

row = 11
col = 10

# Constants
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
SCREEN_TITLE = "Robot Controller"

force_data = None  # or some default value


class JointPosition(Node):
    def spin_in_thread(self):
        """This function will spin the ROS node separately."""
        rclpy.spin(self)

    def __init__(self):
        super().__init__('joint_position')
        self.current_position = None
        self.new_callback_received = False  # New attribute
        self.new_signal_received = False
        self.data = None

        # Initialize variables
        self.motion_type = 1
        self.target_position = [-4.28372736e-03, -4.88533739e-01, 1.57943555e+00, -1.09090181e+00 + math.pi / 2,
                                1.56651260e+00, -1.95721289e-12]
        self.old_target_position = [-4.28372736e-03, -4.88533739e-01, 1.57943555e+00, -1.09090181e+00 + math.pi / 2,
                                    1.56651260e+00, -1.95721289e-12]
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

        # ROS2 Subscription setup to get sensor signal
        self.subscription_sensor = self.create_subscription(Float32MultiArray, '/heatmap_data_topic',
                                                            self.sensor_callback, 10)

        # New ROS2 Service client setup for MyService
        self.my_service_client = self.create_client(SetEvent, "/set_event")
        while not self.my_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SetEvent to become available...')

    def joint_position_callback(self, msg):
        """Callback to process the joint state data from the subscribed topic."""
        # self.get_logger().info(f"Positions: {msg.position}")
        self.current_position = msg.position
        self.new_callback_received = True  # Set the flag to True when new data is received

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

    def sensor_callback(self, signal):
        """Callback to process the sensor data from the subscribed topic."""
        global force_data
        self.data = signal.data
        force_data = signal.data
        self.new_signal_received = True  # Set the flag to True when new data is received

    def send_clear_request(self):
        """Send a clear request using the SetEvent service."""
        clear_request = SetEvent.Request()
        clear_request.func = SetEvent.Request.STOP
        clear_request.arg0 = 0
        clear_request.arg1 = 0
        self.my_service_client.call_async(clear_request)


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
        self.demo_run = False  # New flag for demo function
        self.initial_pos = np.array(
            [-4.28372736e-03, -4.88533739e-01, 1.57943555e+00, -1.09090181e+00 + math.pi / 2,
             1.56651260e+00, -1.95721289e-12])

    def on_draw(self):
        arcade.start_render()

    def on_key_press(self, key, modifiers):
        # Adjust joint angles
        if key in [arcade.key.NUM_1, arcade.key.NUM_3, arcade.key.NUM_5, arcade.key.NUM_7, arcade.key.NUM_9]:
            joint_index = \
                {arcade.key.NUM_1: 0, arcade.key.NUM_3: 1, arcade.key.NUM_5: 2, arcade.key.NUM_7: 3,
                 arcade.key.NUM_9: 4}[
                    key]
            self.angle_increment[joint_index] += 0.05
            print("angle_increment: ", self.angle_increment)

        elif key in [arcade.key.NUM_2, arcade.key.NUM_4, arcade.key.NUM_6, arcade.key.NUM_8, arcade.key.NUM_0]:
            joint_index = \
                {arcade.key.NUM_2: 0, arcade.key.NUM_4: 1, arcade.key.NUM_6: 2, arcade.key.NUM_8: 3,
                 arcade.key.NUM_0: 4}[
                    key]
            self.angle_increment[joint_index] -= 0.05
            print("angle_increment: ", self.angle_increment)

        # Return to starting point when 'o' is pressed
        elif key == arcade.key.O:
            self.run_test = False
            self.joint_node.target_position = [-4.28372736e-03, -4.88533739e-01, 1.57943555e+00,
                                               -1.09090181e+00 + math.pi / 2, 1.56651260e+00, -1.95721289e-12]
            self.joint_node.old_target_position = [-4.28372736e-03, -4.88533739e-01, 1.57943555e+00,
                                                   -1.09090181e+00 + math.pi / 2, 1.56651260e+00, -1.95721289e-12]
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
            print("Current Position: ", self.joint_node.current_position)

        elif key == arcade.key.KEY_1:
            # NEW STUFF
            chain = TM5_Joint_7
            D_Ori = [-5.02137269e-01, -5.02137270e-01, -4.97853556e-01, -4.97853555e-01]

            threading.Thread(target=Skin_Thread, args=()).start()
            print("Skin Thread started")
            threading.Thread(target=self.mujoco_thread, args=(chain, D_Ori)).start()
            print("Mujoco Thread started")
            self.demo_run = True
            print("Demo Run: ", self.demo_run)

        elif key == arcade.key.KEY_2:
            # print("2 key pressed")
            self.demo_run = not self.demo_run
            print("Demo Run: ", self.demo_run)

    def robot_run(self):
        print("Test function started")
        self.counter = 0
        while True:
            if self.run_test:
                if self.joint_node.new_callback_received:
                    self.joint_node.new_callback_received = False
                    self.current_error = np.array(self.joint_node.old_target_position) - np.array(
                        self.joint_node.current_position)
                    print("Current Error: ", self.current_error)

                    error_check = True
                    for i in range(6):
                        threshold = getattr(self, f'error_threshold_{i}')
                        if 0 < threshold <= abs(self.current_error[i]):
                            error_check = False
                            break

                    if error_check:
                        print("Error: ", self.current_error)
                        self.run_test = False  # Reset the flag
                        self.joint_node.send_positions(self.joint_node.target_position)
                        self.joint_node.old_target_position = self.joint_node.target_position
                        print("Test function executed")

    def mujoco_thread(self, chain, D_Ori):
        global alpha
        global wrench_external

        initial_pos = np.array(
            [-4.28372736e-03, -4.88533739e-01, 1.57943555e+00, -1.09090181e+00 + math.pi / 2, 1.56651260e+00,
             -1.95721289e-12])

        # Parameter Init
        t = 0
        arm_desired_twist_adm_ = np.zeros((6, 1))
        desired_pose_position_ = initial_pos
        desired_pose_orientation_ = D_Ori
        links_len = len(chain.links)
        alpha = [-4.28372736e-03, -4.88533739e-01, 1.57943555e+00, -1.09090181e+00 + math.pi / 2, 1.56651260e+00,
                 -1.95721289e-12]

        # start simulation
        for i in range(1000):
            # viewer.sync()
            d.ctrl = initial_pos
            alpha = initial_pos
            mj.mj_step(m, d)
            # time.sleep(0.002)

        with mj.viewer.launch_passive(m, d) as viewer:
            while True:
                if self.demo_run:
                    saved_position = d.qpos  # just newly added, can delete it and switch back to joint_node.current_position

                    viewer.sync()
                    error = np.zeros((6, 1))

                    # Jacobian Calculation
                    joints = np.zeros(9)
                    joints[2:links_len - 1] = d.qpos
                    All_matrix = chain.forward_kinematics(joints, True)[2:links_len - 1]
                    J = calculate_jacobian(All_matrix)

                    # Wrench External Transform
                    Force = wrench_external
                    Force = transform_force_and_torque(Force, All_matrix[-1])
                    wrench_external_ = (J.T @ Force).reshape(-1, 1)
                    wrench_external_ = np.clip(wrench_external_, -3000, 3000)

                    arm_orientation_, arm_position_ = forward_controller(chain,
                                                                         saved_position[
                                                                         0:links_len - 3])
                    arm_position_ = saved_position
                    # arm_position_ = d.qpos

                    arm_desired_twist_adm_, linear_disp, angular_disp, error = admittance_control_fixed(d, error,
                                                                                                        arm_position_,
                                                                                                        desired_pose_position_,
                                                                                                        desired_pose_orientation_,
                                                                                                        arm_orientation_,
                                                                                                        D, M, K,
                                                                                                        arm_desired_twist_adm_,
                                                                                                        m.opt.timestep,
                                                                                                        wrench_external_)

                    """ Calculate New Joint Positions """
                    Current_Joints = saved_position
                    # Current_Joints = d.qpos
                    Delta_Joints = (arm_desired_twist_adm_ * m.opt.timestep).reshape([6, ])  # change the m.opt.timestep to adjust the speed
                    New_Joints = Current_Joints + Delta_Joints
                    New_Joints[3] = -(New_Joints[2] + New_Joints[1] - math.pi / 2)  # <--------- essential, ask binzhi
                    # New_Joints[4] = -(New_Joints[0]-math.pi/2)

                    """ Safety Control """
                    # New_Joints[0] = np.clip(New_Joints[0], -0.5, 0.5)
                    # New_Joints[1] = np.clip(New_Joints[1], -0.6, -0.4)
                    # New_Joints[2] = np.clip(New_Joints[2], 1.3, 2)
                    # New_Joints[3] = np.clip(New_Joints[3], -1.7, -1)
                    # d.ctrl[0:links_len - 3] = New_Joints
                    d.ctrl[0:links_len - 3] = New_Joints
                    alpha = New_Joints

                    mj.mj_step(m, d)


def admittance_control_fixed(d, error,
                             arm_position_,
                             desired_pose_position_,
                             desired_pose_orientation_,
                             arm_orientation_,
                             D, M, K,
                             arm_desired_twist_adm_,
                             dt, wrench_external_):
    arm_position_ = np.array(arm_position_)
    desired_pose_position_ = np.array(desired_pose_position_)
    error = np.around(arm_position_.reshape([6, 1]) - desired_pose_position_.reshape([6, 1]), decimals=3)

    coupling_wrench_arm = D * arm_desired_twist_adm_ + K * error

    arm_desired_acceleration = np.linalg.inv(M) @ (wrench_external_ - coupling_wrench_arm)
    a_acc_norm = np.linalg.norm(arm_desired_acceleration[:3])

    # Acceleration limit check
    if a_acc_norm > arm_max_acc_:
        arm_desired_acceleration[:3] *= arm_max_acc_ / a_acc_norm

    # Update twist admittance
    arm_desired_acceleration = np.around(arm_desired_acceleration[:6], decimals=3)
    arm_desired_twist_adm_ += arm_desired_acceleration * dt

    # Velocity limit check
    a_vel_norm = np.linalg.norm(arm_desired_twist_adm_[:3])
    if a_vel_norm > arm_max_vel_:
        arm_desired_twist_adm_[:3] *= arm_max_vel_ / a_vel_norm
    arm_desired_twist_adm_ = np.around(arm_desired_twist_adm_[:6], decimals=2)

    # Delta linear displacement and angular displacement
    linear_disp = (arm_desired_twist_adm_[:3] * dt).flatten()
    angular_disp = (arm_desired_twist_adm_[3:] * dt).flatten()

    return arm_desired_twist_adm_, linear_disp, angular_disp, error

def forward_controller(chain, angles):
    # Ensure the matrix is a NumPy array
    links_len = len(chain.links)
    forward_input = np.zeros([links_len, ])
    forward_input[2:links_len - 1] = angles
    T = chain.forward_kinematics(forward_input)
    T = np.array(T)

    # Extract the rotation matrix and translation vector
    rotation_matrix = T[:3, :3]
    translation_vector = T[:3, 3]

    # Convert the rotation matrix to Euler angles
    euler_angles = R.from_matrix(rotation_matrix).as_matrix()

    # Translation vector gives the XYZ coordinates
    xyz_coordinates = translation_vector
    return euler_angles, xyz_coordinates


def inverse_controller(chain, pos, euler):
    links_len = len(chain.links)
    result = chain.inverse_kinematics(pos, euler, "all")[2:links_len - 1]
    return result


def calculate_jacobian(T_matrices):
    n = len(T_matrices)
    J = np.zeros((6, n))

    # 末端执行器的位置
    end_effector_pos = T_matrices[-1][:3, 3]

    for i in range(n):
        # 当前关节的旋转矩阵和位置
        R_i = T_matrices[i][:3, :3]
        d_i = T_matrices[i][:3, 3]

        # 计算z轴和p轴
        z = R_i[:, 2]
        p = end_effector_pos - d_i

        # 计算雅克比矩阵的列
        J[:3, i] = np.cross(z, p)
        J[3:, i] = z

    return J


def transform_force_and_torque(Wrench, transformation_matrix):
    local_force = np.array([Wrench[1], Wrench[2], Wrench[0]])  # y, z, x
    local_torque = np.array([Wrench[4], Wrench[5], Wrench[3]])  # ry, rz, rx
    # local_force = Wrench[:3]  # 取前三个元素作为力
    # local_torque = Wrench[3:]  # 取后三个元素作为力矩
    # 提取变换矩阵中的旋转矩阵部分
    rotation_matrix = transformation_matrix[0:3, 0:3]

    # 转换力和力矩
    global_force = rotation_matrix @ local_force
    global_torque = rotation_matrix @ local_torque

    global_force_torque = np.concatenate((global_force, global_torque))

    return global_force_torque


def Skin_Thread():  # <------ never entered
    global wrench_external, force_data
    force_direction = np.zeros((row, col), dtype=object)
    force = np.zeros((row, col), dtype=object)

    while True:
        try:
            data = np.array(force_data).reshape([row, col])
            data = np.clip(data, 0, 2000)

            for i in range(row):
                for j in range(col):
                    force_direction[i, j] = np.array([0,
                                                      -math.cos(2 * math.pi / 2 - j * math.pi / 11),
                                                      -math.sin(2 * math.pi / 2 - j * math.pi / 11)])
                    force_direction[i, j] = np.array([0,
                                                      -math.cos(
                                                          1 * math.pi * 7 / 6 - j * math.pi * 7 / 66 - math.pi / 6),
                                                      -math.sin(
                                                          1 * math.pi * 7 / 6 - j * math.pi * 7 / 66 - math.pi / 6)])
            for i in range(row):
                for j in range(col):
                    force[i, j] = np.array(force_direction[i, j]) * data[i, j]

            # wrench_external = force
            force_y = 0
            force_z = 0
            for i in range(row):
                for j in range(col):
                    force_y += force[i, j][1]  # 累加每个元素的第二个数
                    force_z += force[i, j][2]  # 累加每个元素的第三个数
            wrench_external[1] = force_y
            wrench_external[0] = force_z

            # print("Wrench External: ", wrench_external)

        except Exception as e:
            print(f"Exception in Skin_Thread: {e}")
            # pass

def main(args=None):
    global joint_node
    rclpy.init(args=args)
    joint_node = JointPosition()

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
