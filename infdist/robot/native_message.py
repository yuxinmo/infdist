#! /usr/bin/env python

from datetime import datetime


class ROS2NativeMessage:
    def __init__(self, msg, sender, receivers, data_type_name, data=None,
                 publisher=None):
        self.msg = msg
        self.sender = sender
        self.publisher = publisher
        self.receivers = receivers
        self.t_rcv = datetime.now()
        self.data_type_name = data_type_name
        if data is not None:
            self.data = data
        else:
            self.data = msg
