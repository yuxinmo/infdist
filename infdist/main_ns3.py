from simulator.network import Network
from simulator import simulator
from optimization.models import Message


NODES_NUM = 3

counter = {
    i: 0
    for i in range(NODES_NUM)
}


def gen_message_received(node_id):
    def message_received(message):
        global counter
        print("{} received {}".format(node_id, message))
        print(message.data)
        counter[node_id] += 1

    return message_received


net = Network(NODES_NUM)

for i in range(NODES_NUM):
    net.add_message_received_callback(
        gen_message_received(i),
        i
    )

simulator.schedule(0, lambda: net.send(Message(
    0,
    {1},
    0,
    'batt',
    data={'name': 'michaŁ'}
)))


def get_callback(i):
    def func():
        print('sending', i)
        net.send(Message(
            i % NODES_NUM,
            {1},
            0,
            'batt',
            data={'name': 'm{}'.format(i)}
        ))
    return func


for i in range(100):
    simulator.schedule(1 + i*.00001, get_callback(i))


def print_stats():
    print(counter)


simulator.schedule(3, print_stats)
simulator.run()
