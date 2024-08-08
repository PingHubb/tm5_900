import rclpy
from rclpy.node import Node
from tm_msgs.srv import SendScript
from std_msgs.msg import Float32MultiArray
import sys
import math  # for radian to degree conversion
import time

class SendScriptClient(Node):
    def __init__(self):
        super().__init__('send_script_client')
        self.client = self.create_client(SendScript, 'send_script')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'positions_topic',
            self.positions_callback,
            10)
        self.positions = None

        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(1)
            self.get_logger().info('Waiting for service...')

        """ TEST """
        self.positions_callback(Float32MultiArray(data=[1.0, -4.88533739e-01, 1.57, -1.09090181e+00 + math.pi / 2, 1.57, 0.0]))
        """ TEST """

    def positions_callback(self, msg):
        self.positions = msg.data
        self.get_logger().info(f'Received positions (radians): {self.positions}')
        self.send_and_process_request()

    def send_and_process_request(self):
        if self.positions is None:
            self.get_logger().error('No positions received yet.')
            return

        # Convert positions from radians to a script command
        command = self.convert_positions_to_script(self.positions)
        self.send_request(command)
        # time.sleep(0.1)
        # command = 'StopAndClearBuffer()'
        # self.send_request(command)

    def send_request(self, command):
        request = SendScript.Request()
        request.id = 'demo'  # <-- not important can self-define
        request.script = command
        self.future = self.client.call_async(request)

    def convert_positions_to_script(self, positions):
        # Convert radians to degrees and format as a script command
        positions_deg = [math.degrees(pos) for pos in positions]
        position_str = ','.join(map(str, positions_deg))
        return f"PTP(\"JPP\",{position_str},100,0,100,true)"

    def process_response(self):
        if self.future is None:
            return

        if self.future.done():
            try:
                response = self.future.result()
                if response.ok:
                    self.get_logger().info('Command success')
                else:
                    self.get_logger().info('Command failed')
            except Exception as e:
                self.get_logger().error(f'Service call failed {e}')

def main(args=None):
    rclpy.init(args=args)
    client = SendScriptClient()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            client.process_response()
            # client.future = None  # 重置 future 对象
            # break

    rclpy.shutdown()

if __name__ == '__main__':
    main()

