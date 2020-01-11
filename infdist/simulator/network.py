from collections import defaultdict
import json

import ns.applications
import ns.core
import ns.internet
import ns.network
import ns.mobility
import ns.point_to_point
import ns.wifi

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
        self._message_received_callbacks = defaultdict(set)

        self.nodes = ns.network.NodeContainer()
        self.nodes.Create(nodes_num)

        self._init_network()

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
        for i, socket in enumerate(self.sockets):
            socket.SetRecvCallback(self._gen_ns3_receive_callback(i))
            socket.SetAllowBroadcast(True)
            socket.Bind(any_address)
            socket.Connect(broadcast_address)

    def _init_network(self):
        rss = -80
        phy_mode = "DsssRate1Mbps"

        wifi = ns.wifi.WifiHelper()
        wifi.SetStandard(ns.wifi.WIFI_PHY_STANDARD_80211b)

        wifi_phy = ns.wifi.YansWifiPhyHelper.Default()
        wifi_phy.Set("RxGain", ns.core.DoubleValue(0))
        wifi_phy.SetPcapDataLinkType(
            ns.wifi.YansWifiPhyHelper.DLT_IEEE802_11_RADIO
        )

        wifi_channel = ns.wifi.YansWifiChannelHelper()
        wifi_channel.SetPropagationDelay(
            "ns3::ConstantSpeedPropagationDelayModel"
        )
        # fixed RSS regardless of the distance and transmit power
        wifi_channel.AddPropagationLoss(
            "ns3::FixedRssLossModel",
            "Rss",
            ns.core.DoubleValue(rss)
        )

        wifi_phy.SetChannel(wifi_channel.Create())

        # disable rate control
        wifi.SetRemoteStationManager(
            "ns3::ConstantRateWifiManager",
            "DataMode", ns.core.StringValue(phy_mode),
            "ControlMode", ns.core.StringValue(phy_mode),
        )

        wifi_mac = ns.wifi.WifiMacHelper()
        wifi_mac.SetType("ns3::AdhocWifiMac")

        # mobility
        mobility = ns.mobility.MobilityHelper()
        positions = ns.mobility.ListPositionAllocator()
        for i in range(self.nodes.GetN()):
            positions.Add(ns.mobility.Vector(5*i, int(i/5)*2, 0))

        mobility.SetPositionAllocator(positions)
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel")
        mobility.Install(self.nodes)

        devices = wifi.Install(wifi_phy, wifi_mac, self.nodes)

        self._init_ip(devices)

    def _init_ip(self, devices):
        stack = ns.internet.InternetStackHelper()
        stack.Install(self.nodes)

        address = ns.internet.Ipv4AddressHelper()
        address.SetBase(ns.network.Ipv4Address("10.1.1.0"),
                        ns.network.Ipv4Mask("255.255.255.0"))

        address.Assign(devices)

    def add_message_received_callback(self, callback, node_id):
        self._message_received_callbacks[node_id].add(callback)

    def _message_received(self, message, node_id):
        for callback in self._message_received_callbacks[node_id]:
            callback(message)

    def _header2message(self, header, receivers):
        return Message(
            header.GetSender(),
            receivers=receivers,
            planned_t_sent=None,
            data_type_name=self.known_data_types.get_data_type_name(
                header.GetDataType()),
            data=json.loads(header.GetData()),
            t_sent=header.GetTimestamp()/10**9,
            t_rcv=simulator.now()/10**9,
        )

    def _gen_ns3_receive_callback(self, node_id):
        def _ns3_receive_callback(socket):
            packet = socket.Recv()
            header = ns.network.InfDistHeader()
            packet.RemoveHeader(header)
            # print("rcvd", self._header_repr(header), ns.core.Simulator.Now())
            self._message_received(
                self._header2message(header, {node_id}),
                node_id,
            )
        return _ns3_receive_callback

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
