import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


from std_msgs.msg import String

from .mission_manager import MissionClient, MissionExecutor


class MissionMessagePublisher(Node, MissionExecutor):
    def __init__(self):
        super().__init__('mission_message_publisher')
        self.publisher_ = self.create_publisher(String, 'topic')
        """
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        """

    def start_mission(self, timestamp):
        self.get_logger().info('starting mission @ {}.{}'.format(
            timestamp.sec,
            timestamp.nanosec,
        ))

    def end_mission(self, timestamp):
        self.get_logger().info('ending mission @ {}.{}'.format(
            timestamp.sec,
            timestamp.nanosec,
        ))


def main(args=None):
    rclpy.init(args=args)

    node = MissionMessagePublisher()
    mission_client = MissionClient()
    mission_client.add_mission_executor(node)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(mission_client)

    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
