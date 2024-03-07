import serial
import time
import sys
import select

class ArduinoCommander:
    def __init__(self, serial_port, baud_rate):
        try:
            self.ser = serial.Serial(serial_port, baud_rate)
        except serial.SerialException:
            print(f"Serial port {serial_port} not found.")
            sys.exit(1)

    def send_command(self, command):
        full_command = command + '\n'
        self.ser.write(full_command.encode())

    def channel_check(self):
        self.send_command("channelCheck")

    def read_cal(self):
        self.send_command("readCal")

    def update_cal(self):
        self.send_command("updateCal")

    def read_raw(self):
        self.send_command("readRaw")

    def close(self):
        self.ser.close()

    def read_response(self):
        if self.ser.in_waiting > 0:
            return self.ser.readline().decode('utf-8').rstrip()
        return None

def main():
    serial_port = '/dev/ttyACM0'  # Replace with your serial port name
    baud_rate = 9600  # In arduino, Serial.begin(baud_rate)

    commander = ArduinoCommander(serial_port, baud_rate)

    print("Enter commands to send to Arduino (type 'exit' to quit):")

    while True:
        try:
            if select.select([sys.stdin], [], [], 0)[0]:
                user_input = sys.stdin.readline().strip().lower()
                if user_input == 'exit':
                    break

                start_time = time.time()

                if user_input == 'channelcheck':
                    commander.channel_check()
                elif user_input == 'readcal':
                    commander.read_cal()
                elif user_input == 'updatecal':
                    commander.update_cal()
                elif user_input == 'readraw':
                    commander.read_raw()
                else:
                    print("Unknown command.")

            response = commander.read_response()
            if response:
                end_time = time.time()
                elapsed_time = end_time - start_time
                print(response)
                print(f"Response time: {elapsed_time:.4f} seconds")

        except KeyboardInterrupt:
            print("Interrupted by user")
            break
        except Exception as e:
            print(f"Error: {e}")
            break

    commander.close()

if __name__ == '__main__':
    main()
