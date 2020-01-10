from simulator.network import Network
from simulator import simulator
from optimization.models import Message


def message_received(message):
    print(message)
    print(message.data)


net = Network(3)
net.add_message_received_callback(message_received)

simulator.schedule(0, lambda: net.send(Message(
    0,
    {1},
    0,
    'batt',
    data={'name': 'michaŁ'}
)))

simulator.schedule(1, lambda: net.send(Message(
    1,
    {1},
    0,
    'batt',
    data={'name': 'michaŁ'}
)))


simulator.schedule(2, lambda: net.send(Message(
    2,
    {1},
    0,
    'batt',
    data={'name': 'michaŁ'}
)))

simulator.run()
