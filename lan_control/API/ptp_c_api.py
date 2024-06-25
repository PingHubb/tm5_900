from tm_msgs.srv import SetPositions
from rclpy.node import Node

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.client = self.create_client(SetPositions, "/set_positions")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SetPositions service to become available...')

    def send_positions(self, positions, velocity, acc_time, blend_percentage, fine_goal):
        self.get_logger().info('Sending position request...')
        request = SetPositions.Request()
        request.motion_type = 2  # PTP_T
        request.positions = [float(pos) for pos in positions]
        request.velocity = float(velocity)
        request.acc_time = float(acc_time)
        request.blend_percentage = int(blend_percentage)
        request.fine_goal = fine_goal

        future = self.client.call_async(request)
        return future