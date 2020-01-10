import numpy as np

from aggregations import AggregationMax
from models import MissionContext, MessageSet, Message, InformationType
from utilities import UtilityBattery, UtilityPosition

BATTERY_DATA_TYPE = 'batt'
POSITION_DATA_TYPE = 'position'


def generate_periodic_messages(
    t_end, senders, receivers, data_type_name,
    t_start=0, f=1, data_f=lambda t: {}
):
    if receivers is None:
        receivers = senders
    return MessageSet(t_end, [
        Message(
            agent_id,
            set(receivers) - set([agent_id]),
            t,
            data_type_name,
            data_f(t)
        )
        for agent_id in senders
        for t in np.arange(t_start, t_end, 1/f)
    ], t_start)


def generate_batt_messages(t_end, senders, receivers, t_start=0, f=1,
                           level_start=1, level_end=0):

    def batt_level(t):
        a = (level_start - level_end) / (t_start - t_end)
        b = level_end - a * t_end
        level = a*t+b
        return {
            'battery_level': level,
        }

    return (
        generate_periodic_messages(
            t_end, senders, receivers,
            BATTERY_DATA_TYPE, t_start, f, batt_level
        ),
        MissionContext(
            set([
                InformationType(
                    BATTERY_DATA_TYPE,
                    utility_cls=UtilityBattery,
                    aggregation_cls=AggregationMax,
                )
            ])
        )
    )


def generate_pos_messages(t_end, senders, receivers, t_start=0, f=5):
    return (
        generate_periodic_messages(
            t_end, senders, receivers, POSITION_DATA_TYPE, t_start, f
        ),
        MissionContext(set([
            InformationType(
                POSITION_DATA_TYPE,
                utility=UtilityPosition,
                aggregation=AggregationMax,
            )
        ]))
    )


def simulate_sending_messages_with_latency(msgs, latency):
    for m in msgs.all():
        if m.t_sent is None:
            m.t_sent = m.planned_t_sent
        m.t_rcv = m.t_sent + latency


def generate_simple_3D_reconstruction(
    t_end, senders={0, 1}, receivers=None,
):
    return (
        generate_batt_messages(t_end, senders, receivers, level_end=0.88)
        # + generate_pos_messages(t_end, senders, receivers)
        # + generate_status_messages(t_end, senders, receivers)
        # + generate_objective_messages(t_end, senders, receivers)
        # + generate_map_messages(t_end, senders, receivers)
    )


if __name__ == "__main__":
    msgs, context = generate_simple_3D_reconstruction(22)
    print(msgs.messages.__str__(
        ['sender', 'receivers', 'planned_t_sent', 't_sent', 'data',
         'data_type_name']))
