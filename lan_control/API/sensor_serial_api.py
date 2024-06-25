import serial
import sys
import time


class ArduinoCommander:
    def __init__(self, serial_ports, baud_rate):
        self.serial_ports = serial_ports
        self.baud_rate = baud_rate
        self.serial_connections = []

        for port in self.serial_ports:
            try:
                ser = serial.Serial(port, baud_rate)
                print(f"Serial port {port}  opened successfully.")
                self.serial_connections.append(ser)
            except serial.SerialException:
                print(f"Serial port {port} not found.")

    def send_command(self, command, port_index=0):
        responses = []
        for ser in self.serial_connections:
            full_command = command + '\n'
            ser.write(full_command.encode())
            response = self.read_response(ser, command)
            responses.append(response)
            # print(f"Sent '{command}' to {ser.port}, received: {response}")
        # print(f"Responses: {responses}")
        return responses

    def channel_check(self, port_index=0):
        return self.send_command("channelCheck", port_index)

    def update_cal(self, port_index=0):
        return self.send_command("updateCal", port_index)

    def read_cal(self, port_index=0):
        return self.send_command("readCal", port_index)

    def read_raw(self, port_index=0):
        return self.send_command("readRaw", port_index)

    def stop(self, port_index=0):
        return self.send_command("stop", port_index)

    def close(self):
        for ser in self.serial_connections:
            ser.close()

    def read_response(self, ser, command, timeout=2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if ser.in_waiting > 0:
                response = ser.readline().decode('utf-8').rstrip()
                data_list = [int(value) for value in response.split() if value.isdigit()]
                if data_list:
                    if command == "readRaw":
                        return data_list[2:-2]
                    elif command == "readCal":
                        return data_list[2:-2]
                    elif command == "updateCal":
                        return data_list[2:-2]
                    elif command == "channelCheck":
                        return data_list
        return None

