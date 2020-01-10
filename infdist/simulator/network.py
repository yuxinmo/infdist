import json

import ns.applications
import ns.core
import ns.internet
import ns.network
import ns.point_to_point

from . import simulator
from optimization.models import Message


class DataTypesStore:
    def __init__(self):
        self.data_types = {}
        self.hashes = {}

    def get_hash(self, data_type_name):
        if data_type_name not in self.data_types:
            h = bytes([len(self.data_types)])
            self.data_types[data_type_name] = h
            self.hashes[h] = data_type_name
        return self.data_types[data_type_name]

    def get_data_type_name(self, h):
        return self.hashes[h]


class Network:
    DATA_PORT = 4477

    def __init__(self, nodes_num):
        self.known_data_types = DataTypesStore()
        self._message_received_callbacks = set()

        self.nodes = ns.network.NodeContainer()
        self.nodes.Create(nodes_num)

        pointToPoint = ns.point_to_point.PointToPointHelper()
        pointToPoint.SetDeviceAttribute(
            "DataRate", ns.core.StringValue("5Mbps")
        )
        pointToPoint.SetChannelAttribute(
            "Delay", ns.core.StringValue("2000ms")
        )

        devices = pointToPoint.Install(self.nodes)

        stack = ns.internet.InternetStackHelper()
        stack.Install(self.nodes)

        address = ns.internet.Ipv4AddressHelper()
        address.SetBase(ns.network.Ipv4Address("10.1.1.0"),
                        ns.network.Ipv4Mask("255.255.255.0"))

        self.interfaces = address.Assign(devices)
        self.sockets = [
            ns.network.Socket.CreateSocket(
                self.nodes.Get(i),
                ns.core.TypeId.LookupByName("ns3::UdpSocketFactory")
            )
            for i in range(nodes_num)
        ]

        any_address = ns.network.InetSocketAddress(
            ns.network.Ipv4Address.GetAny(), self.DATA_PORT
        )
        broadcast_address = ns.network.InetSocketAddress(
            ns.network.Ipv4Address.GetBroadcast(), self.DATA_PORT
        )
        for socket in self.sockets:
            socket.SetRecvCallback(self._ns3_receive_callback)
            socket.SetAllowBroadcast(True)
            socket.Bind(any_address)
            socket.Connect(broadcast_address)

    def add_message_received_callback(self, callback):
        self._message_received_callbacks.add(callback)

    def _message_received(self, message):
        for callback in self._message_received_callbacks:
            callback(message)

    def _header2message(self, header):
        return Message(
            header.GetSender(),
            receivers=None,
            planned_t_sent=None,
            data_type_name=self.known_data_types.get_data_type_name(
                header.GetDataType()),
            data=json.loads(header.GetData()),
            t_sent=header.GetTimestamp()/10**9,
            t_rcv=simulator.now()/10**9,
        )

    def _ns3_receive_callback(self, socket):
        packet = socket.Recv()
        header = ns.network.InfDistHeader()
        packet.RemoveHeader(header)
        # print("received", self._header_repr(header), ns.core.Simulator.Now())
        self._message_received(
            self._header2message(header)
        )

    @staticmethod
    def _header_repr(header):
        return '{}@{} from {}; data="{}"'.format(
            header.GetDataType(),
            header.GetTimestamp(),
            header.GetSender(),
            header.GetData(),
        )

    def send(self, message):
        print("sending", ns.core.Simulator.Now())
        packet = ns.network.Packet(1024)

        header = ns.network.InfDistHeader()
        header.SetDataType(
            self.known_data_types.get_hash(message.data_type_name)
        )
        header.SetTimestamp(simulator.now())
        header.SetSender(message.sender)
        data = json.dumps(message.data)
        header.SetData(len(data), data)
        packet.AddHeader(header)
        self.sockets[message.sender].Send(packet)
