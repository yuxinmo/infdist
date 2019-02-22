from datetime import datetime, timedelta
import pickle
import sys

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from .mission_manager import (
    MissionClient, MissionExecutor
)
from .mission_generator.missions import generate_simple_3D_reconstruction
from .mission_generator.models import MessageSet
from .time_utils import datetime_from_time_msg
from .message_converter import msg_to_ros_msg, ros_msg_to_msg
from comm_evaluation_msgs.msg import EvaluationHeader


class MissionMessagePublisher(Node, MissionExecutor):
    def __init__(self, messages):
        super().__init__('mission_message_publisher')
        self.publisher_ = self.create_publisher(EvaluationHeader, 'topic')
        self.messages = messages
        self.current_message = 0
        self.timer = None

    def _message_real_sent_time(self, msg):
        return self.mission_start_time + timedelta(seconds=msg.planned_t_sent)

    def publish_current_message(self):

        msg = self.messages.all()[self.current_message]

        publish_delay = (
            datetime.now() - self._message_real_sent_time(msg)
        ).total_seconds()

        if publish_delay > 0.01:
            self.get_logger().warn(
                'High publish delay: {}. Publisher overloaded?'.format(
                    publish_delay
                )
            )

        msg.t_sent = datetime.now()

        ros_msg = msg_to_ros_msg(msg)

        self.publisher_.publish(ros_msg)
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
            max(wait_time.total_seconds(), 0), self.timer_callback
        )

    def start_mission(self, timestamp):
        self.get_logger().info('starting mission @ {}.{}'.format(
            timestamp.sec,
            timestamp.nanosec,
        ))
        self.current_message = 0
        self.mission_start_time = datetime_from_time_msg(timestamp)
        if self.messages.all():
            self.set_timer()
        else:
            self.get_logger().warn('empty messages set')

    def end_mission(self, timestamp):
        self.get_logger().info('ending mission @ {}.{}'.format(
            timestamp.sec,
            timestamp.nanosec,
        ))
        self.destroy_timer(self.timer)


class MissionMessageEvaluator(Node, MissionExecutor):
    def __init__(self, receiver_ident, message_types):
        super().__init__('mission_message_evaluator')
        self.receiver_ident = receiver_ident
        self.message_types = message_types
        self.subscription = self.create_subscription(
            EvaluationHeader,
            'topic',
            self.listener_callback
        )

    def listener_callback(self, ros_msg):
        msg = ros_msg_to_msg(ros_msg)
        msg.t_rcv = datetime.now()

        if self.receiver_ident not in msg.receivers:
            return

        self.messages.append(msg)

    def start_mission(self, timestamp):
        self.get_logger().info('starting mission @ {}.{}'.format(
            timestamp.sec,
            timestamp.nanosec,
        ))
        self.mission_start = datetime_from_time_msg(timestamp)
        self.messages = MessageSet(1, [], t_start=self.mission_start)

    def end_mission(self, timestamp):
        self.get_logger().info('ending mission @ {}.{}'.format(
            timestamp.sec,
            timestamp.nanosec,
        ))
        self.messages.t_end = datetime_from_time_msg(timestamp)
        self.log_mission_summary()
        pickle.dump(
            self.messages,
            open('logs/d{}.pickle'.format(self.receiver_ident), 'wb')
        )

    def aggregate_types(self):
        result = {}
        for type_name, message_type in self.message_types.items():
            msgs = self.messages.filter(data_type=type_name)
            self.get_logger().info('all msgs of type {}: {}'.format(
                type_name, msgs))

            agg = message_type.aggregation(message_type.utility(), msgs)
            result[type_name] = agg.integrate()
        return result

    def log_mission_summary(self):
        M = len(self.messages.all())
        text = ''

        for msg in self.messages.all():
            text += '\t{}\n'.format(repr(msg))

        delays = [msg.t_rcv - msg.t_sent for msg in self.messages.all()]
        delay_mean = sum(delays, timedelta()) / M
        delay_variance = sum([(delay-delay_mean).total_seconds()**2
                              for delay in delays]) / M

        text += ' ====== \n'
        text += 'Mission start: {} Mission end: {}\n'.format(
            self.mission_start, self.messages.t_end
        )
        text += 'Total messages received: {}\n'.format(M)
        text += 'Delay mean: {} variance: {}\n'.format(
            delay_mean, delay_variance
        )

        text += 'Aggregated utility: {}\n'.format(self.aggregate_types())

        self.get_logger().info(text)


def main(args=None):
    if len(sys.argv) > 1:
        ident = int(sys.argv[1])
    else:
        ident = 1

    rclpy.init(args=args)

    msgs, message_types = generate_simple_3D_reconstruction(5)

    mission_publisher = MissionMessagePublisher(
        msgs.filter(sender=ident)
    )
    mission_subscriber = MissionMessageEvaluator(
        ident,
        message_types
    )

    mission_client = MissionClient()
    mission_client.add_mission_executor(mission_publisher)
    mission_client.add_mission_executor(mission_subscriber)

    executor = SingleThreadedExecutor()
    executor.add_node(mission_publisher)
    executor.add_node(mission_subscriber)
    executor.add_node(mission_client)

    mission_publisher.get_logger().info(
        'Node #{} initialized, waiting for a mission start.'.format(ident)
    )

    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
