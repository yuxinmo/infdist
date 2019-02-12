from datetime import datetime, timedelta

from std_msgs.msg import Header
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from .mission_manager import MissionClient, MissionExecutor, get_current_time
from .mission_generator.models import Message, MessageSet


class MissionMessagePublisher(Node, MissionExecutor):
    def __init__(self):
        super().__init__('mission_message_publisher')
        self.publisher_ = self.create_publisher(Header, 'topic')
        self.messages = MessageSet(20, [
            Message(0, [1], 1, None),
            Message(0, [1], 6, None),
        ])
        self.current_message = 0
        self.timer = None

    def _message_real_sent_time(self, msg):
        return self.mission_start_time + timedelta(seconds=msg.t_sent)

    def publish_current_message(self):
        msg = Header()
        msg.stamp = get_current_time()
        self.publisher_.publish(msg)
        self.current_message += 1

    def timer_callback(self):
        self.destroy_timer(self.timer)
        now = datetime.now()
        while (
            self.current_message < len(self.messages.all())
            and
            self._message_real_sent_time(
                self.messages.all()[self.current_message]
            ) < now
        ):
            self.publish_current_message()

        if self.current_message < len(self.messages.all()):
            self.set_timer()
        else:
            self.get_logger().info('mission finished')

    def set_timer(self):
        msg = self.messages.all()[self.current_message]
        wait_time = (
            self._message_real_sent_time(msg) - datetime.now()
        )

        if self.timer is not None:
            self.destroy_timer(self.timer)
        self.timer = self.create_timer(
            wait_time.total_seconds(), self.timer_callback
        )

    def start_mission(self, timestamp):
        self.get_logger().info('starting mission @ {}.{}'.format(
            timestamp.sec,
            timestamp.nanosec,
        ))
        self.current_message = 0
        self.mission_start_time = datetime.fromtimestamp(
            timestamp.sec + timestamp.nanosec * 1e-9
        )
        self.set_timer()

    def end_mission(self, timestamp):
        self.get_logger().info('ending mission @ {}.{}'.format(
            timestamp.sec,
            timestamp.nanosec,
        ))
        self.destroy_timer(self.timer)


class MissionMessageSubscriber(Node, MissionExecutor):
    def __init__(self):
        super().__init__('mission_message_subscriber')
        self.subscription = self.create_subscription(
            Header,
            'topic',
            self.listener_callback
        )

    def listener_callback(self, msg):
        print('Got message {}'.format(msg))

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

    mission_publisher = MissionMessagePublisher()
    mission_subscriber = MissionMessageSubscriber()

    mission_client = MissionClient()
    mission_client.add_mission_executor(mission_publisher)
    mission_client.add_mission_executor(mission_subscriber)

    executor = SingleThreadedExecutor()
    executor.add_node(mission_publisher)
    executor.add_node(mission_subscriber)
    executor.add_node(mission_client)

    mission_publisher.get_logger().info(
        'Node initialized, waiting for a mission start.'
    )

    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
