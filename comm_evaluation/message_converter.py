from datetime import datetime
import math

from builtin_interfaces.msg import Time

from comm_evaluation_msgs.msg import EvaluationHeader
from .mission_generator.models import Message


# TODO implement this function
# and change it to ros_topic_to_data_type
def ros_type_to_type(ros_type):
    return 'batt'


def type_to_ros_type(msg_type):
    return EvaluationHeader


def ros_msg_to_msg(ros_msg):
    msg = Message(
        ros_msg.sender_id,
        ros_msg.receivers,
        None,
        ros_type_to_type(type(ros_msg)),
        t_sent=datetime.fromtimestamp(
            ros_msg.stamp.sec + ros_msg.stamp.nanosec * 1e-9
        ),
        data={'battery_level': 1},
    )
    return msg


def msg_to_ros_msg(msg):
    ros_msg = type_to_ros_type(msg.data_type)()
    ros_msg.sender_id = msg.sender
    ros_msg.receivers = msg.receivers
    ros_msg.stamp = _datetime_to_ros_time(msg.t_sent)
    return ros_msg


def _datetime_to_ros_time(datetime):
    modf = math.modf(datetime.timestamp())
    nanosec, sec = map(int, (modf[0]*10**9, modf[1]))
    return Time(sec=sec, nanosec=nanosec)
