#! /usr/bin/env python

from infdist.optimization.network import BaseNetwork
from infdist.optimization.models import Message


class ROS2NativeMessage:
    def __init__(self, msg, sender, receivers, data_type_name,
                 data, stamp, t_rcv=None, publisher=None):
        self.msg = msg
        self.sender = sender
        self.publisher = publisher
        self.receivers = receivers
        self.t_rcv = t_rcv
        self.data_type_name = data_type_name
        self.data = data
        self.stamp = stamp


class ROS2Network(BaseNetwork):
    def deserialize(self, native_message):
        # if native_message.msg:
        #     t_gen = (
        #         native_message.msg.header.stamp.sec +
        #         native_message.msg.header.stamp.nanosec * 1e-9
        #     )
        t_gen = native_message.stamp
        return Message(
            sender=native_message.sender,
            receivers=native_message.receivers,
            t_gen=t_gen,
            data_type_name=native_message.data_type_name,
            data=native_message.data,
        )

    def send(self, native_message):
        native_message.publisher.publish(native_message.msg)
