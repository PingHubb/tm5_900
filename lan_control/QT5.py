import sys
import threading
import rclpy
import transforms3d
import time
import os
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLineEdit, QLabel, QPushButton, QStackedLayout
from PyQt5.QtCore import pyqtSlot
from std_msgs.msg import Float64MultiArray
from lan_control.API import ptp_c_api
from lan_control.OLD import sensor_serial_api_old

# Serial Communication Setup
serial_port = '/dev/ttyACM0'
baud_rate = 9600

sensor_read_count = 720
# speed = 0.4715
speed = 0.25  # Remember robot speed is 10%
sensor_count = 3000
time_sleep = 1

class UI(QWidget):
    def __init__(self):
        super().__init__()
        self.robot_controller = None
        self.init_main_ui()
        self.init_robot_ui()
        self.layout = QStackedLayout(self)
        self.layout.addWidget(self.main_container)
        self.layout.addWidget(self.robot_container)
        self.setLayout(self.layout)
        self.sensor_api = sensor_serial_api_old.ArduinoCommander(serial_port, baud_rate)
        self.x_y_step = 0.0075

    def init_main_ui(self):
        self.main_container = QWidget()
        main_layout = QVBoxLayout(self.main_container)
        self.connect_to_robot = QPushButton("Connect to Robot", self)
        self.connect_to_robot.clicked.connect(self.switch_to_robot_ui)
        main_layout.addWidget(self.connect_to_robot)
        self.B = QPushButton("B", self)
        main_layout.addWidget(self.B)
        self.C = QPushButton("C", self)
        main_layout.addWidget(self.C)

    def init_robot_ui(self):
        self.robot_container = QWidget()
        robot_layout = QVBoxLayout(self.robot_container)

        # Layouts for positions and quaternions
        position_layout = QHBoxLayout()
        quaternion_layout = QHBoxLayout()
        button_layout = QHBoxLayout()

        # Define position fields
        self.position_x = QLineEdit("-0.462", self)
        self.position_y = QLineEdit("-0.571", self)
        self.position_z = QLineEdit("0.35", self)
        position_layout.addWidget(QLabel("Position X:"))
        position_layout.addWidget(self.position_x)
        position_layout.addWidget(QLabel("Position Y:"))
        position_layout.addWidget(self.position_y)
        position_layout.addWidget(QLabel("Position Z:"))
        position_layout.addWidget(self.position_z)

        # Define quaternion fields
        self.quaternion_x = QLineEdit("0.0", self)
        self.quaternion_y = QLineEdit("1.0", self)
        self.quaternion_z = QLineEdit("0.0", self)
        self.quaternion_w = QLineEdit("0.0", self)
        quaternion_layout.addWidget(QLabel("Quaternion X:"))
        quaternion_layout.addWidget(self.quaternion_x)
        quaternion_layout.addWidget(QLabel("Quaternion Y:"))
        quaternion_layout.addWidget(self.quaternion_y)
        quaternion_layout.addWidget(QLabel("Quaternion Z:"))
        quaternion_layout.addWidget(self.quaternion_z)
        quaternion_layout.addWidget(QLabel("Quaternion W:"))
        quaternion_layout.addWidget(self.quaternion_w)

        # Buttons for control
        self.send_button = QPushButton("Send to Robot", self)
        self.send_button.clicked.connect(self.send_to_robot_button)
        button_layout.addWidget(self.send_button)
        self.send_to_region_a = QPushButton("Region A", self)
        self.send_to_region_a.clicked.connect(self.region_A)
        button_layout.addWidget(self.send_to_region_a)
        self.send_to_region_b = QPushButton("Region B", self)
        self.send_to_region_b.clicked.connect(self.region_B)
        button_layout.addWidget(self.send_to_region_b)
        self.send_to_region_c = QPushButton("Region C", self)
        self.send_to_region_c.clicked.connect(self.region_C)
        button_layout.addWidget(self.send_to_region_c)
        self.all_measurment_in_one = QPushButton("All in one", self)
        self.all_measurment_in_one.clicked.connect(self.All_in_one)
        button_layout.addWidget(self.all_measurment_in_one)
        self.experiment_1cm = QPushButton("Experiment 2 (1cm)", self)
        self.experiment_1cm.clicked.connect(self.experiment_2_1cm)
        button_layout.addWidget(self.experiment_1cm)
        self.experiment_2cm = QPushButton("Experiment 2 (2cm)", self)
        self.experiment_2cm.clicked.connect(self.experiment_2_2cm)
        button_layout.addWidget(self.experiment_2cm)
        self.experiment_xcm = QPushButton("Experiment 2 (Xcm)", self)
        self.experiment_xcm.clicked.connect(self.experiment_2_xcm)
        button_layout.addWidget(self.experiment_xcm)

        # Add all layouts to the main robot layout
        robot_layout.addLayout(position_layout)
        robot_layout.addLayout(quaternion_layout)
        robot_layout.addLayout(button_layout)
        self.robot_container.setVisible(False)  # Initially hidden

    def start_ros(self):
        rclpy.init()
        self.robot_controller = ptp_c_api.RobotController()
        rclpy.spin(self.robot_controller)

    def print_data(self):
        data = self.sensor_api.read_raw()
        print(data)
        return data

    @pyqtSlot()
    def switch_to_robot_ui(self):
        self.layout.setCurrentWidget(self.robot_container)
        if not self.robot_controller:
            self.ros_thread = threading.Thread(target=self.start_ros)
            self.ros_thread.start()

    @pyqtSlot()
    def send_to_robot_button(self):
        positions = [
            float(self.position_x.text()),
            float(self.position_y.text()),
            float(self.position_z.text())
        ]
        quaternion = [
            float(self.quaternion_x.text()),
            float(self.quaternion_y.text()),
            float(self.quaternion_z.text()),
            float(self.quaternion_w.text())
        ]
        euler = transforms3d.euler.quat2euler(quaternion, axes='sxyz')
        full_position = positions + list(euler)

        velocity = speed
        acc_time = 0.0
        blend_percentage = 100
        fine_goal = False

        future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage, fine_goal)

    def experiment_2_xcm(self):
        print("Experiment 2")
        distance = 3.9

        for i in range(51):
            a = i + 30
            positions = [-0.486, -0.567, 0.4]
            quaternion = (0.0, 1.0, 0.0, 0.0)
            euler = transforms3d.euler.quat2euler(quaternion, axes='sxyz')
            full_position = positions + list(euler)
            velocity = speed
            acc_time = 0.0
            blend_percentage = 100
            fine_goal = False

            formatted_distance = f"{distance:.1f}"
            file_location_Xcm = f'/home/ping2/ros2_ws/src/lan_control/lan_control/resources/experiment_2/{formatted_distance}cm/sensor_readings_experiment_2_{formatted_distance}cm.txt'
            distance += 0.1

            direction = 1  # Start moving downward initially
            cell = 0

            future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage, fine_goal)
            time.sleep(4)

            with open(file_location_Xcm, 'w') as file:
                file.write(f"--- Reference Data ---\n")
                for i in range(sensor_count):
                    sensor_data = self.print_data()
                    data_str = ', '.join(map(str, sensor_data)) if isinstance(sensor_data, list) else str(
                        sensor_data)
                    file.write(data_str + "\n")
            print("Printed reference data")
            time.sleep(time_sleep)

            print(f"Starting experiment for {distance}cm")

            positions[2] = 0.284 + a * 0.001
            full_position = positions + list(euler)
            future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage, fine_goal)
            time.sleep(1)

            with open(file_location_Xcm, 'a') as file:
                file.write(f"--- Path: {cell} ---\n")
                for i in range(sensor_count):
                    sensor_data = self.print_data()
                    data_str = ', '.join(map(str, sensor_data)) if isinstance(sensor_data, list) else str(
                        sensor_data)
                    file.write(data_str + "\n")
            print(f"Printed data for path {cell}")
            cell += 1
            time.sleep(time_sleep)

            # Traverse through each row in the grid
            for layer in range(5):  # Loop through each layer
                for step in range(4):  # 5 moves per column because it's a 5x5 grid
                    if direction > 0:
                        positions[0] += self.x_y_step  # Adjust X coordinate for downward
                        positions[1] += self.x_y_step  # Adjust Y coordinate for downward
                    else:
                        positions[0] -= self.x_y_step  # Adjust X coordinate for upward
                        positions[1] -= self.x_y_step  # Adjust Y coordinate for upward
                    full_position = positions + list(euler)
                    future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage,
                                                                  fine_goal)

                    time.sleep(1)

                    with open(file_location_Xcm, 'a') as file:
                        file.write(f"--- Path: {cell} ---\n")
                        for i in range(sensor_count):
                            sensor_data = self.print_data()
                            data_str = ', '.join(map(str, sensor_data)) if isinstance(sensor_data, list) else str(
                                sensor_data)
                            file.write(data_str + "\n")
                    print(f"Printed data for path {cell}")
                    cell += 1
                    time.sleep(time_sleep)


                if layer < 4:  # Check to prevent an extra horizontal movement at the last iteration
                    positions[0] += self.x_y_step  # Increment X coordinate for left move
                    positions[1] -= self.x_y_step  # Decrement Y coordinate for left move
                    full_position = positions + list(euler)
                    future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage,
                                                                  fine_goal)

                    time.sleep(1)

                    with open(file_location_Xcm, 'a') as file:
                        file.write(f"--- Path: {cell} ---\n")
                        for i in range(sensor_count):
                            sensor_data = self.print_data()
                            data_str = ', '.join(map(str, sensor_data)) if isinstance(sensor_data, list) else str(
                                sensor_data)
                            file.write(data_str + "\n")
                    print(f"Printed data for path {cell}")
                    cell += 1
                    time.sleep(time_sleep)

                direction *= -1  # Change direction for next column's vertical movement

        print(f"Experiment 2: {distance} completed")


    def experiment_2_2cm(self):
        print("Experiment 2")
        positions = [-0.486, -0.567, 0.335]
        quaternion = (0.0, 1.0, 0.0, 0.0)
        euler = transforms3d.euler.quat2euler(quaternion, axes='sxyz')
        full_position = positions + list(euler)
        velocity = speed
        acc_time = 0.0
        blend_percentage = 100
        fine_goal = False

        file_location_2cm = '/home/ping2/ros2_ws/src/lan_control/lan_control/resources/experiment_2/2cm/sensor_readings_experiment_2_2cm.txt'

        direction = 1  # Start moving downward initially
        cell = 0

        future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage, fine_goal)
        time.sleep(4)

        with open(file_location_2cm, 'w') as file:
            file.write(f"--- Reference Data ---\n")
            for i in range(sensor_count):
                sensor_data = self.print_data()
                data_str = ', '.join(map(str, sensor_data)) if isinstance(sensor_data, list) else str(
                    sensor_data)
                file.write(data_str + "\n")
        print("Printed reference data")
        time.sleep(time_sleep)

        positions[2] = 0.295
        full_position = positions + list(euler)
        future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage, fine_goal)
        time.sleep(1)

        with open(file_location_2cm, 'a') as file:
            file.write(f"--- Path: {cell} ---\n")
            for i in range(sensor_count):
                sensor_data = self.print_data()
                data_str = ', '.join(map(str, sensor_data)) if isinstance(sensor_data, list) else str(
                    sensor_data)
                file.write(data_str + "\n")
        print(f"Printed data for path {cell}")
        cell += 1
        time.sleep(time_sleep)

        # Traverse through each row in the grid
        for layer in range(5):  # Loop through each layer
            for step in range(4):  # 5 moves per column because it's a 5x5 grid
                if direction > 0:
                    positions[0] += self.x_y_step  # Adjust X coordinate for downward
                    positions[1] += self.x_y_step  # Adjust Y coordinate for downward
                else:
                    positions[0] -= self.x_y_step  # Adjust X coordinate for upward
                    positions[1] -= self.x_y_step  # Adjust Y coordinate for upward
                full_position = positions + list(euler)
                future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage,
                                                              fine_goal)

                time.sleep(1)

                with open(file_location_2cm, 'a') as file:
                    file.write(f"--- Path: {cell} ---\n")
                    for i in range(sensor_count):
                        sensor_data = self.print_data()
                        data_str = ', '.join(map(str, sensor_data)) if isinstance(sensor_data, list) else str(
                            sensor_data)
                        file.write(data_str + "\n")
                print(f"Printed data for path {cell}")
                cell += 1
                time.sleep(time_sleep)


            if layer < 4:  # Check to prevent an extra horizontal movement at the last iteration
                positions[0] += self.x_y_step  # Increment X coordinate for left move
                positions[1] -= self.x_y_step  # Decrement Y coordinate for left move
                full_position = positions + list(euler)
                future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage,
                                                              fine_goal)

                time.sleep(1)

                with open(file_location_2cm, 'a') as file:
                    file.write(f"--- Path: {cell} ---\n")
                    for i in range(sensor_count):
                        sensor_data = self.print_data()
                        data_str = ', '.join(map(str, sensor_data)) if isinstance(sensor_data, list) else str(
                            sensor_data)
                        file.write(data_str + "\n")
                print(f"Printed data for path {cell}")
                cell += 1
                time.sleep(time_sleep)

            direction *= -1  # Change direction for next column's vertical movement

        print("Experiment 2 completed")

    @pyqtSlot()
    def experiment_2_1cm(self):
        print("Experiment 2")
        positions = [-0.486, -0.567, 0.335]
        quaternion = (0.0, 1.0, 0.0, 0.0)
        euler = transforms3d.euler.quat2euler(quaternion, axes='sxyz')
        full_position = positions + list(euler)
        velocity = speed
        acc_time = 0.0
        blend_percentage = 100
        fine_goal = False

        file_location_1cm = '/home/ping2/ros2_ws/src/lan_control/lan_control/resources/experiment_2/1cm/sensor_readings_experiment_2_1cm.txt'

        direction = 1  # Start moving downward initially
        cell = 0

        future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage, fine_goal)
        time.sleep(4)

        with open(file_location_1cm, 'w') as file:
            file.write(f"--- Reference Data ---\n")
            for i in range(sensor_count):
                sensor_data = self.print_data()
                data_str = ', '.join(map(str, sensor_data)) if isinstance(sensor_data, list) else str(
                    sensor_data)
                file.write(data_str + "\n")
        print("Printed reference data")
        time.sleep(time_sleep)

        positions[2] = 0.282
        full_position = positions + list(euler)
        future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage, fine_goal)
        time.sleep(1)

        with open(file_location_1cm, 'a') as file:
            file.write(f"--- Path: {cell} ---\n")
            for i in range(sensor_count):
                sensor_data = self.print_data()
                data_str = ', '.join(map(str, sensor_data)) if isinstance(sensor_data, list) else str(
                    sensor_data)
                file.write(data_str + "\n")
        print(f"Printed data for path {cell}")
        cell += 1
        time.sleep(time_sleep)

        # Traverse through each row in the grid
        for layer in range(5):  # Loop through each layer
            for step in range(4):  # 5 moves per column because it's a 5x5 grid
                if direction > 0:
                    positions[0] += self.x_y_step  # Adjust X coordinate for downward
                    positions[1] += self.x_y_step  # Adjust Y coordinate for downward
                else:
                    positions[0] -= self.x_y_step  # Adjust X coordinate for upward
                    positions[1] -= self.x_y_step  # Adjust Y coordinate for upward
                full_position = positions + list(euler)
                future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage,
                                                              fine_goal)

                time.sleep(1)

                with open(file_location_1cm, 'a') as file:
                    file.write(f"--- Path: {cell} ---\n")
                    for i in range(sensor_count):
                        sensor_data = self.print_data()
                        data_str = ', '.join(map(str, sensor_data)) if isinstance(sensor_data, list) else str(
                            sensor_data)
                        file.write(data_str + "\n")
                print(f"Printed data for path {cell}")
                cell += 1
                time.sleep(time_sleep)


            if layer < 4:  # Check to prevent an extra horizontal movement at the last iteration
                positions[0] += self.x_y_step  # Increment X coordinate for left move
                positions[1] -= self.x_y_step  # Decrement Y coordinate for left move
                full_position = positions + list(euler)
                future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage,
                                                              fine_goal)

                time.sleep(1)

                with open(file_location_1cm, 'a') as file:
                    file.write(f"--- Path: {cell} ---\n")
                    for i in range(sensor_count):
                        sensor_data = self.print_data()
                        data_str = ', '.join(map(str, sensor_data)) if isinstance(sensor_data, list) else str(
                            sensor_data)
                        file.write(data_str + "\n")
                print(f"Printed data for path {cell}")
                cell += 1
                time.sleep(time_sleep)

            direction *= -1  # Change direction for next column's vertical movement

        print("Experiment 2 completed")

    @pyqtSlot()
    def All_in_one(self):
        self.region_A()
        print("Region A Done")
        self.region_B()
        print("Region B Done")
        self.region_C()
        print("Region C Done")

    @pyqtSlot()
    def region_A(self):
        positions = [-0.486, -0.567, 0.335]
        quaternion = (0.0, 1.0, 0.0, 0.0)
        euler = transforms3d.euler.quat2euler(quaternion, axes='sxyz')
        full_position = positions + list(euler)
        velocity = speed
        acc_time = 0.0
        blend_percentage = 100
        fine_goal = False

        future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage, fine_goal)
        time.sleep(4)

        positions = [-0.486, -0.567, 0.282]
        full_position = positions + list(euler)

        future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage, fine_goal)

        with open('/home/ping2/ros2_ws/src/lan_control/sensor_readings_A.txt', 'w') as file:
            file.write(f"--- Step: Moving Down ---\n")
            for i in range(sensor_read_count):
                sensor_data = self.print_data()
                data_str = ', '.join(map(str, sensor_data)) if isinstance(sensor_data, list) else str(sensor_data)
                file.write(data_str + "\n")

        time.sleep(1)

        positions = [-0.486, -0.567, 0.335]
        full_position = positions + list(euler)

        future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage, fine_goal)

        with open('/home/ping2/ros2_ws/src/lan_control/sensor_readings_A.txt', 'a') as file:
            file.write(f"--- Step: Moving Up ---\n")
            for i in range(sensor_read_count):
                sensor_data = self.print_data()
                data_str = ', '.join(map(str, sensor_data)) if isinstance(sensor_data, list) else str(sensor_data)
                file.write(data_str + "\n")

    @pyqtSlot()
    def region_B(self):
        positions = [-0.471, -0.567, 0.335]
        quaternion = (0.0, 1.0, 0.0, 0.0)
        euler = transforms3d.euler.quat2euler(quaternion, axes='sxyz')
        full_position = positions + list(euler)
        velocity = speed
        acc_time = 0.0
        blend_percentage = 100
        fine_goal = False

        future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage, fine_goal)
        time.sleep(4)

        positions = [-0.471, -0.567, 0.282]
        full_position = positions + list(euler)

        future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage, fine_goal)

        with open('/home/ping2/ros2_ws/src/lan_control/sensor_readings_B.txt', 'w') as file:
            file.write(f"--- Step: Moving Down ---\n")
            for i in range(sensor_read_count):
                sensor_data = self.print_data()
                data_str = ', '.join(map(str, sensor_data)) if isinstance(sensor_data, list) else str(sensor_data)
                file.write(data_str + "\n")

        time.sleep(1)

        positions = [-0.471, -0.567, 0.335]
        full_position = positions + list(euler)

        future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage, fine_goal)

        with open('/home/ping2/ros2_ws/src/lan_control/sensor_readings_B.txt', 'a') as file:
            file.write(f"--- Step: Moving Up ---\n")
            for i in range(sensor_read_count):
                sensor_data = self.print_data()
                data_str = ', '.join(map(str, sensor_data)) if isinstance(sensor_data, list) else str(sensor_data)
                file.write(data_str + "\n")

    @pyqtSlot()
    def region_C(self):
        positions = [-0.456, -0.567, 0.335]
        quaternion = (0.0, 1.0, 0.0, 0.0)
        euler = transforms3d.euler.quat2euler(quaternion, axes='sxyz')
        full_position = positions + list(euler)
        velocity = speed
        acc_time = 0.0
        blend_percentage = 100
        fine_goal = False

        future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage, fine_goal)
        time.sleep(4)

        positions = [-0.456, -0.567, 0.282]
        full_position = positions + list(euler)

        future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage, fine_goal)

        with open('/home/ping2/ros2_ws/src/lan_control/sensor_readings_C.txt', 'w') as file:
            file.write(f"--- Step: Moving Down ---\n")
            for i in range(sensor_read_count):
                sensor_data = self.print_data()
                data_str = ', '.join(map(str, sensor_data)) if isinstance(sensor_data, list) else str(sensor_data)
                file.write(data_str + "\n")

        time.sleep(1)

        positions = [-0.456, -0.567, 0.335]
        full_position = positions + list(euler)

        future = self.robot_controller.send_positions(full_position, velocity, acc_time, blend_percentage, fine_goal)

        with open('/home/ping2/ros2_ws/src/lan_control/sensor_readings_C.txt', 'a') as file:
            file.write(f"--- Step: Moving Up ---\n")
            for i in range(sensor_read_count):
                sensor_data = self.print_data()
                data_str = ', '.join(map(str, sensor_data)) if isinstance(sensor_data, list) else str(sensor_data)
                file.write(data_str + "\n")

    def closeEvent(self, event):
        if self.robot_controller:
            self.robot_controller.destroy_node()
            rclpy.shutdown()
        event.accept()


def main():
    app = QApplication(sys.argv)
    ex = UI()
    ex.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
