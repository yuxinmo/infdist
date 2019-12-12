from datetime import datetime
import math

from builtin_interfaces.msg import Time

from infdist_msgs.msg import EvaluationHeader
from .mission_generator.models import Message


ros_topic_types = {
    'battery': EvaluationHeader,
    'position': EvaluationHeader,
}

msg_type_to_ros_topic = {
    'batt': 'battery',
    'position': 'position',
}

ros_topic_to_msg_type = {
    ros_topic: msg_type
    for msg_type, ros_topic in msg_type_to_ros_topic.items()
}


def ros_topic_to_type(ros_topic):
    return ros_topic_to_msg_type[ros_topic]


def type_to_ros_type(msg_type):
    topic = msg_type_to_ros_topic[msg_type]
    return ros_topic_types[topic], topic


def ros_msg_to_msg(ros_msg, ros_topic):
    msg = Message(
        ros_msg.sender_id,
        ros_msg.receivers,
        None,
        ros_topic_to_type(ros_topic),
        t_sent=datetime.fromtimestamp(
            ros_msg.stamp.sec + ros_msg.stamp.nanosec * 1e-9
        ),
        data={'battery_level': 1},
    )
    return msg


def msg_to_ros_msg(msg):
    ros_msg_class, ros_topic = type_to_ros_type(msg.data_type)

    ros_msg = ros_msg_class()
    ros_msg.sender_id = msg.sender
    ros_msg.receivers = msg.receivers
    ros_msg.stamp = _datetime_to_ros_time(msg.t_sent)
    return ros_msg, ros_topic


def _datetime_to_ros_time(datetime):
    modf = math.modf(datetime.timestamp())
    nanosec, sec = map(int, (modf[0]*10**9, modf[1]))
    return Time(sec=sec, nanosec=nanosec)
