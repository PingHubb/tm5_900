import socket
import select
import time
import numpy as np
import struct
import rclpy
import sys

from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String

buffer = []
storage = []
sublist = []
Calibration_values = []
dis = []
global new_list
new_list = []
separated_lists = []
global averages
averages = []  # Initialize it
touchDiffData = []
size = 1
counter = 0  # global counter
bb = 1
restarted = 0
total = 0
number = 1
count_1 = 0
count_2 = 0

should_stop_readRaw = False
should_stop_readCal = False
channelDrive_global = None
channelSensor_global = None
should_stop_readRaw_print = True
publisher = None
ip = "10.0.0.109" # 185 / 58 / 47 / 109

def send_command(command, ip, port):
    global client_socket

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        client_socket.connect((ip, port))
        client_socket.setblocking(True)
        print("Connected")

        if command == "channelCheck":
            client_socket.send(command.encode())
            print("--------------------", flush=True)
            print(f"Sending command: {command}", flush=True)
            print("--------------------", flush=True)
            channelCheck(client_socket)
            client_socket.close()
            print("Connection closed")


        elif command == "readCal":
            client_socket.send(command.encode())
            print("--------------------", flush=True)
            print(f"Sending command: {command}", flush=True)
            print("--------------------", flush=True)
            readCal(client_socket)
            client_socket.close()
            print("Connection closed")

        elif command == "updateCal":
            client_socket.send(command.encode())
            print("--------------------", flush=True)
            print(f"Sending command: {command}", flush=True)
            print("--------------------", flush=True)
            updateCal()
            client_socket.close()
            print("Connection closed")


        elif command == "readRaw":
            client_socket.send(command.encode())
            print("--------------------", flush=True)
            print(f"Sending command: {command}", flush=True)
            print("--------------------", flush=True)
            readRaw(client_socket)

        elif command == "getDistance":
            print("--------------------", flush=True)
            print(f"Sending command: {command}", flush=True)
            print("--------------------", flush=True)
            getDistance()

        elif command == "oneKey":
            print("--------------------", flush=True)
            print(f"Sending command: {command}", flush=True)
            print("--------------------", flush=True)
            one_key()

        elif command == "stop":
            client_socket.send(command.encode())
            print("--------------------", flush=True)
            print(f"Sending command: {command}", flush=True)
            print("--------------------", flush=True)
            client_socket.close()
            print("Connection closed")

        else:
            print("Unknown Command!!!", flush=True)

    except Exception as e:
        print(f"Error: {e}", flush=True)
    finally:
        print(f"cycle end: {command}", flush=True)
        # client_socket.close()

def channelCheck(client_socket):
    global channelDrive_global, channelSensor_global

    client_socket.setblocking(True)
    response = client_socket.recv(12)

    unpacked_data = struct.unpack('<6H', response)
    print(unpacked_data, flush=True)
    special_code_1, special_code_2, channelDrive, channelSensor, special_code_3, special_code_4 = unpacked_data
    if special_code_1 == 55555 and special_code_2 == 55555 and special_code_3 == 44444 and special_code_4 == 44444:
        print(f"Channels Ready: Driver = {channelDrive}, Sensor = {channelSensor}", flush=True)
    else:
        print("Data not formatted correctly or special codes mismatch", flush=True)
        client_socket.close()
        time.sleep(0.5)
        send_command("channelCheck", ip, 80)

    channelDrive_global = channelDrive
    channelSensor_global = channelSensor

    return channelDrive, channelSensor

def readCal(client_socket):
    global Calibration_values, no_of_elements, data_in_bytes, channelDrive_global, channelSensor_global

    count = 1
    no_of_elements = channelDrive_global * channelSensor_global

    data_in_bytes = (no_of_elements + 4) * 2

    while True:
        if should_stop_readCal:
            break
        ready, _, _ = select.select([client_socket], [], [], 1.0)
        if ready:
            data = client_socket.recv(data_in_bytes)
            hex_values = [data[i:i + 2] for i in range(0, len(data), 2)]
            full_values = [int(hex_value[::-1].hex(), 16) for hex_value in hex_values]
            Calibration_values = full_values[2:-2]
            # print(f"readCal:", count,  Calibration_values, flush=True)
            count += 1
        if count == 6: # <-------- if you don't want to press the stop button after readCal, uncomment this two lines
            print(f"readCal:", count, full_values, flush=True)
            print(f"Length of data received: {len(Calibration_values)}", flush=True)

            break      # <-------- if you don't want to press the stop button after readCal, uncomment this two lines

def updateCal():
    print("--------------------", flush=True)
    print("Updated Calibration", flush=True)
    print("--------------------", flush=True)

def readRaw(client_socket):
    global buffer, should_stop_readRaw, storage, sublist, list_storage, new_list, averages, separated_lists, should_stop_everything, restarted, total, should_stop_readRaw_print, number

    one_time_flag = True
    count = 1
    should_stop_everything = False

    while should_stop_everything is False:
            if should_stop_readRaw:
                break
            ready, _, _ = select.select([client_socket], [], [], 1.0)
            if ready:
                data = client_socket.recv(data_in_bytes)
                hex_values = [data[i:i + 2] for i in range(0, len(data), 2)]
                decimal_values = [int(hex_value[::-1].hex(), 16) for hex_value in hex_values]
                buffer.extend(decimal_values)   # Save the data into buffer list

                if len(data) == 0 or len(data) is None:
                    print("Received 'Please Reconnect' message from Arduino. Attempting to reconnect...")
                    total = total + count
                    restarted += 1
                    number = 1
                    send_command("readRaw", ip, 80)

                if one_time_flag:
                    while len(buffer) >= 2 and (buffer[0] != 55555 or buffer[1] != 55555):
                        buffer.pop(0)
                    one_time_flag = len(buffer) < 2 or buffer[0] != 55555 or buffer[1] != 55555
                if len(buffer) < no_of_elements + 4:
                    continue
                sublist = buffer[:no_of_elements + 4]
                if is_valid_sublist(sublist):
                    storage.extend(sublist[2:no_of_elements + 2])
                    buffer = buffer[no_of_elements + 4:]
                    new_list.extend(sublist[2:no_of_elements + 2])
                    if len(new_list) >= size * no_of_elements:
                        for i in range(0, len(new_list), no_of_elements):
                            separated_list = new_list[i:i + no_of_elements]
                            separated_lists.append(separated_list)
                        averages = []
                        for elements in zip(*separated_lists):
                            avg_value = sum(elements) / size
                            averages.append(avg_value)
                        new_list = []
                        separated_lists = []
                    matrix_readRaw = []
                    for i in range(0, len(sublist[2:no_of_elements + 2]), channelSensor_global):
                        row = sublist[2 + i: 2 + i + channelSensor_global]
                        matrix_readRaw.append(row)
                    if should_stop_readRaw_print is False:
                        print(f"Package:", count, "Restarted:", restarted, "Total:", total, "Length of averages:", len(averages), flush=True)
                        for row in matrix_readRaw:
                            print(row, flush=True)
                else:
                    index_first_55555 = find_first_55555_pair(sublist)
                    if index_first_55555 is not None:
                        buffer = buffer[index_first_55555:]
                    else:
                        print("Error occurs! Error occurs! Error occurs!!!", flush=True)

                # sublist = sublist[2:-2]
                # if count % size == 0:
                #     getDistance()
                #     should_stop_readRaw_print = True

                count += 1

def find_first_55555_pair(sublist):
    return next((i for i in range(2, len(sublist) - 2) if sublist[i] == 55555 and sublist[i + 1] == 55555), None)

def is_valid_sublist(sublist):
    return sublist[:2] == [55555, 55555] and sublist[-2:] == [44444, 44444] and 55555 not in sublist[2:-2] and 44444 not in sublist[2:-2]

def getDistance():
    global touchDiffData, dis, should_stop_everything, averages, sublist, Calibration_values, restarted, total, number

    should_stop_everything = False

    if should_stop_readRaw is False:
        if len(Calibration_values) == 0:
            print("Calibration values are empty.", flush=True)

        if len(averages) == 0:
            print("Averages are empty.", flush=True)

        if len(Calibration_values) != len(averages):
            print(f"Mismatch in lengths: Calibration_values ({len(Calibration_values)}) vs Averages ({len(averages)})", flush=True)

        touchDiffData = np.array(Calibration_values) - np.array(averages)
        touchDiffPerc = (10000 * touchDiffData) / np.array(Calibration_values)
        dis = touchDiffPerc
        dis[touchDiffPerc < 15] = -1
        dis[(touchDiffPerc >= 15) & (touchDiffPerc < 20)] = 5
        dis[(touchDiffPerc >= 20) & (touchDiffPerc < 30)] = 4
        dis[(touchDiffPerc >= 30) & (touchDiffPerc < 50)] = 3
        dis[(touchDiffPerc >= 50) & (touchDiffPerc < 70)] = 2
        dis[(touchDiffPerc >= 70)] = 1
        if len(averages) == 0:
            print("sublist is empty", flush=True)
        elif len(Calibration_values) == 0 or len(averages) == 0:
            print("Calibration is empty.", flush=True)

        # dis = dis[:-10] # This is used to del the last unwanted elements
        # print(f"Package:", number, "Restarted:", restarted, "Total:", total, flush=True)
        # print(dis)
        number += 1

        # print("Entering send_ist.......:", count, flush=True) # This must not be deleted!!!!!!! will lag
        send_list()

def send_list():
    global publisher, bb, restarted, total, channelDrive_global, channelSensor_global, count_1, count_2

    msg_list = Float32MultiArray()

    # Convert dis to a  matrix
    matrix = np.reshape(dis, (channelDrive_global, channelSensor_global))

    # Transpose the matrix
    transposed_matrix = np.transpose(matrix)
    flipped_matrix = np.flipud(transposed_matrix)

    # Flatten the 2D list back to a 1D list
    dis_updated = [elem for row in flipped_matrix for elem in row]

    # Convert dis list to float and assign to msg_list.data
    msg_list.data = [float(x) for x in dis_updated]

    # Count occurrences of 1 and 2 separately
    count_1 = np.count_nonzero(flipped_matrix == 1) + count_1
    count_2 = np.count_nonzero(flipped_matrix == 2) + count_2

    print(f"Package:", bb, "Restarted:", restarted, "Total:", total, "(Here is send list)", flush=True)
    print(f"Count of 1: {count_1}, Count of 2: {count_2}", flush=True)  # Print the count of 1s and 2s
    print(flipped_matrix, flush=True) # < --- print this, this is real
    # print(matrix, flush=True)

    publisher.publish(msg_list)
    bb += 1

def one_key():
    global should_stop_readRaw, should_stop_readRaw_print, should_stop_readCal, client_socket

    print("--------------------", flush=True)
    print('Entering one_key process..........')
    print("--------------------", flush=True)

    # time.sleep(3)
    should_stop_readRaw = False
    should_stop_readRaw_print = False

    send_command("stop", ip,80)
    time.sleep(1.5)

    send_command("channelCheck", ip,80)
    time.sleep(1.5)

    send_command("readCal", ip, 80)
    time.sleep(1.5)

    send_command("updateCal", ip, 80)
    time.sleep(1.5)

    # threading.Thread(target=send_command, args=("readRaw", ip, 80)).start()
    send_command("readRaw", ip, 80)
    time.sleep(1.5)

    # should_stop_readRaw_print = True
    # threading.Thread(target=send_command, args=("getDistance", ip, 80)).start()
    # time.sleep(1.5)

    # threading.Thread(target=check_for_keypress).start()

def main():
    global publisher

    rclpy.init()
    node = Node('my_node_one_key')
    publisher = node.create_publisher(Float32MultiArray, 'my_send_list', 10)

    # app = QApplication(sys.argv)
    # window = QWidget()
    # window.setWindowTitle('PyQt5 App')
    # window.setFixedSize(400, 300)  # Set the fixed window size to 400x300 pixels
    # layout = QVBoxLayout()
    # button = QPushButton('Click me')
    # button.clicked.connect(button_clicked)  # Connect the button's click signal to the button_clicked function
    # layout.addWidget(button)
    # window.setLayout(layout)
    # window.show()

    one_key()
    rclpy.spin(node)

if __name__ == '__main__':
    main()