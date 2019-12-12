import argparse
from datetime import datetime, timedelta
import json
import os
import pickle

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from mission_manager.client import (
    MissionClient, MissionExecutor
)
from .mission_generator.missions import generate_simple_3D_reconstruction
from .mission_generator.models import MessageSet
from .message_converter import (
    msg_to_ros_msg,
    ros_msg_to_msg,
    ros_topic_types,
)


class MissionMessagePublisher(Node, MissionExecutor):
    def __init__(self, messages, qos_profile=None):
        super().__init__('mission_message_publisher')
        self.publishers = {
            topic_name: self.create_publisher(topic_type, topic_name,
                                              qos_profile=qos_profile)
            for topic_name, topic_type in ros_topic_types.items()
        }

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

        ros_msg, ros_topic_name = msg_to_ros_msg(msg)

        self.publishers[ros_topic_name].publish(ros_msg)
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
        self.get_logger().info('starting mission @ {}'.format(timestamp))
        self.current_message = 0
        self.mission_start_time = timestamp
        if self.messages.all():
            self.set_timer()
        else:
            self.get_logger().warn('empty messages set')

    def end_mission(self, timestamp):
        self.get_logger().info('ending mission @ {}'.format(timestamp))
        self.destroy_timer(self.timer)


class MissionMessageEvaluator(Node, MissionExecutor):
    def __init__(self, receiver_ident, experiment_configuration, message_types,
                 qos_profile=None, results_file_prefix=None):
        super().__init__('mission_message_evaluator')
        self.receiver_ident = receiver_ident
        self.message_types = message_types
        self.results_file_prefix = results_file_prefix

        for topic_name, topic_type in ros_topic_types.items():
            self.create_subscription(
                topic_type, topic_name,
                self.get_subscription_callback(
                    topic_name
                ),
                qos_profile=qos_profile,
            )
        self.mission_start = None

    def get_subscription_callback(self, topic_name):
        def _listener_callback(ros_msg):
            if self.mission_start is None:
                self.get_logger().warn(
                    'Received a message before mission start, ignoring.'
                )
                return

            msg = ros_msg_to_msg(ros_msg, topic_name)
            msg.t_rcv = datetime.now()

            if self.receiver_ident not in msg.receivers:
                return

            self.messages.append(msg)
        return _listener_callback

    def start_mission(self, timestamp):
        self.get_logger().info('starting mission @ {}'.format(timestamp))
        self.mission_start = timestamp
        self.messages = MessageSet(1, [], t_start=self.mission_start)

    def end_mission(self, timestamp):
        self.get_logger().info('ending mission @ {}'.format(timestamp))
        self.messages.t_end = timestamp
        self.print_mission_summary()
        if self.results_file_prefix is not None:
            self.save_results_to_file(self.results_file_prefix)

    def save_results_to_file(self, filename_prefix):
        os.makedirs(os.path.dirname(filename_prefix), exist_ok=True)
        pickle.dump(
            self.messages,
            open(filename_prefix + '.pickle', 'wb')
        )
        self.get_logger().info('Saving results to file {}'.format(
            filename_prefix + '.pickle'
        ))
        json.dump(
            self.aggregate_types(),
            open(filename_prefix + '.agg.json', 'w')
        )

    def aggregate_types(self):
        result = {}
        for sender in sorted(self.messages.senders()):
            result[sender] = {}
            for type_name, message_type in self.message_types.items():
                msgs = self.messages.filter(
                    data_type=type_name,
                    sender=sender,
                )

                agg = message_type.aggregation(message_type.utility(), msgs)
                result[sender][type_name] = agg.integrate()
            return result

    def print_mission_summary(self):
        M = len(self.messages.all())
        text = ''

        # for msg in self.messages.all():
        #     text += '\t{}\n'.format(repr(msg))

        delays = [msg.t_rcv - msg.t_sent for msg in self.messages.all()]

        if M == 0:
            delay_mean = 0
            delay_variance = 0
        else:
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


RELIABILITY_BY_ARG = {
    'reliable': QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    'best_effort': QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
}

DURABILITY_BY_ARG = {
    'transient_local':
        QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    'volatile': QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
}


def history_by_arg(arg):
    if arg == 0:
        return QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL
    else:
        return QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Run message publisher and communication evaluator.')
    parser.add_argument('node_id', type=int, help='node id')

    parser.add_argument(
        '--history', default=10, type=int,
        help='a value for the history QoS parameter, 0 equals to KEEP_ALL'
    )
    parser.add_argument(
        '--reliability', default='reliable', type=str,
        choices=RELIABILITY_BY_ARG.keys(),
        help='a value for the reliability QoS parameter'
    )
    parser.add_argument(
        '--durability', default='volatile', type=str,
        choices=DURABILITY_BY_ARG.keys(),
        help='a value for the durability QoS parameter'
    )
    parser.add_argument(
        '--length', default=100, type=int,
        help='length of a generated mission'
    )
    parser.add_argument(
        '--agents_num', default=2, type=int,
        help='number of agents taking part in the mission'
    )
    parser.add_argument(
        '--results_file_prefix', default=None, type=str,
        help=(
            'a path to a file where the results should be saved, '
            'a dot and extension will be attached to this value '
            '(e.g. ".pickle")'
        )
    )

    parsed_args = parser.parse_args()

    experiment_configuration = {
        'qos': {
            'history': parsed_args.history,
            'reliability': parsed_args.reliability,
            'durability': parsed_args.durability,
        }
    }

    qos_profile = QoSProfile(
        reliability=RELIABILITY_BY_ARG[parsed_args.reliability],
        history=history_by_arg(parsed_args.history),
        depth=parsed_args.history,
        durability=DURABILITY_BY_ARG[parsed_args.durability]
    )

    rclpy.init(args=args)

    msgs, message_types = generate_simple_3D_reconstruction(
        parsed_args.length,
        parsed_args.agents_num,
    )

    mission_publisher = MissionMessagePublisher(
        msgs.filter(sender=parsed_args.node_id),
        qos_profile=qos_profile
    )
    mission_subscriber = MissionMessageEvaluator(
        parsed_args.node_id,
        experiment_configuration,
        message_types,
        qos_profile=qos_profile,
        results_file_prefix=parsed_args.results_file_prefix
    )

    mission_client = MissionClient()
    mission_client.add_mission_executor(mission_publisher)
    mission_client.add_mission_executor(mission_subscriber)

    executor = SingleThreadedExecutor()
    executor.add_node(mission_publisher)
    executor.add_node(mission_subscriber)
    executor.add_node(mission_client)

    mission_publisher.get_logger().info(
        'Node #{} initialized, waiting for a mission start.'.format(
            parsed_args.node_id
        )
    )
    mission_publisher.get_logger().info(
        'Experiment configuration: {}'.format(str(experiment_configuration))
    )

    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
