import numpy as np

from .aggregations import AggregationMax
from .models import MessageSet, Message, MessageType
from .utilities import UtilityBattery, UtilityPosition

BATTERY_DATA_TYPE = 'batt'
POSITION_DATA_TYPE = 'position'


def generate_periodic_messages(
    t_end, agents_num, data_type, t_start=0, f=1, data_f=lambda t: {}
):
    return MessageSet(t_end, [
        Message(
            agent_id,
            set(range(agents_num)) - set([agent_id]),
            t,
            data_type,
            data_f(t)
        )
        for agent_id in range(agents_num)
        for t in np.arange(t_start, t_end, 1/f)
    ])


def generate_batt_messages(t_end, agents_num, t_start=0, f=1,
                           level_start=1, level_end=0):

    def batt_level(t):
        a = (level_start - level_end) / (t_start - t_end)
        b = level_end - a * t_end
        level = a*t+b
        return {
            'batt_level': level,
        }

    return MessageType(
        BATTERY_DATA_TYPE,
        messages=generate_periodic_messages(
            t_end, agents_num, BATTERY_DATA_TYPE, t_start, f, batt_level),
        utility=UtilityBattery,
        aggregation=AggregationMax,
    )


def generate_pos_messages(t_end, agents_num, t_start=0, f=5):
    return MessageType(
        POSITION_DATA_TYPE,
        messages=generate_periodic_messages(
            t_end, agents_num, POSITION_DATA_TYPE, t_start, f),
        utility=UtilityPosition,
        aggregation=AggregationMax,
    )


def generate_simple_3D_reconstruction(
    t_end, agents_num=2,
):
    data_types = [
        generate_batt_messages(t_end, agents_num, level_end=0.88),
        # generate_pos_messages(t_end, agents_num),
        # generate_status_messages(t_end),
        # generate_objective_messages(t_end),
        # generate_map_messages(t_end),
    ]

    type_by_name = {
        _type.name: _type
        for _type in data_types
    }
    messages = sum([_type.messages for _type in data_types], MessageSet(0, []))

    return messages, type_by_name


if __name__ == "__main__":
    msgs = generate_simple_3D_reconstruction(22)
    print(msgs.__str__(
        ['sender', 'receivers', 'planned_t_sent', 't_sent', 'data',
         'data_type']))
