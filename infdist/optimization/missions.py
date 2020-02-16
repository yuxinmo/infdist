import numpy as np

from .aggregations import AggregationMostRecent
from .models import MissionContext, MessageSet, Message, InformationType
from .utilities import UtilityBattery, UtilityPosition
from .message_forecast import FullKnowledgeTypeForecast, EmptyTypeForecast  # NOQA

BATTERY_DATA_TYPE = 'batt'
POSITION_DATA_TYPE = 'position'


def generate_periodic_messages(
    t_end, senders, receivers, data_type_name,
    t_start=0, f=1, data_f=lambda t: {}, sigma_t=0,
    append_sender_to_data_type_name=False,
):
    def gen_data_type_name(sender):
        if append_sender_to_data_type_name:
            return data_type_name + str(sender)
        else:
            return data_type_name

    if receivers is None:
        receivers = senders
    return MessageSet(t_end, [
        Message(
            agent_id,
            set(receivers) - set([agent_id]),
            abs(np.random.normal(t, sigma_t)),
            gen_data_type_name(agent_id),
            data_f(t)
        )
        for agent_id in senders
        for t in np.arange(t_start, t_end, 1/f)
    ], t_start)


def generate_batt_messages(t_end, senders, receivers, t_start=0, f=1,
                           level_start=1, level_end=0, sigma_t=0,
                           max_depl_rate_mi=0.003, max_depl_rate_sigma=0.002
                           ):

    # mi = np.random.normal(0.016, 0.01)
    randomized = False
    if randomized:
        mi = np.random.normal(0.2, 0.01)
    else:
        mi = np.random.normal(0.2, 0.1)

    def batt_level(t):
        a = (level_start - level_end) / (t_start - t_end)
        b = level_end - a * t_end
        level = a*t+b
        return {
            'battery_level': level,
            'max_depl_rate': max(
                0.003, np.random.normal(mi, 0.002)
            ) if randomized else mi
        }

    messages = generate_periodic_messages(
        t_end, senders, receivers,
        BATTERY_DATA_TYPE, t_start, f, batt_level,
        sigma_t,
        append_sender_to_data_type_name=True,
    )
    return (
        messages,
        MissionContext(
            set([
                InformationType(
                    data_type_name,
                    utility_cls=UtilityBattery,
                    aggregation_cls=AggregationMostRecent,
                    # message_forecast_cls=FullKnowledgeTypeForecast,
                    # message_forecast_kwargs={
                    #     'messages': messages,
                    # },
                    message_forecast_cls=EmptyTypeForecast,
                    message_forecast_kwargs={
                        't_end': messages.t_end,
                    },
                    weight=np.random.random()*10+0.5
                )
                for data_type_name in set(
                    m.data_type_name
                    for m in messages.all()
                )

            ])
        )
    )


def generate_pos_messages(t_end, senders, receivers, t_start=0, f=5,
                          sigma_t=0):
    messages = generate_periodic_messages(
        t_end, senders, receivers, POSITION_DATA_TYPE, t_start, f,
        sigma_t,
        append_sender_to_data_type_name=True,
    )
    return (
        messages,
        MissionContext(set([
            InformationType(
                m.data_type_name,
                utility=UtilityPosition,
                aggregation=AggregationMostRecent,
            )
            for m in messages.all()
        ]))
    )


def generate_simple_3D_reconstruction(
    t_end, senders={0, 1}, receivers=None, sigma_t=0.01, seed=2,
):
    if receivers is None:
        receivers = senders

    np.random.seed(seed)

    all_messages = MessageSet(t_end, [])
    all_contexts = MissionContext(set())

    for sender in senders:
        level_start = np.random.random()
        level_end = level_start * np.random.random()
        messages, context = generate_batt_messages(
            t_end, {sender}, receivers,
            # f=(np.random.random()+.2),
            f=np.random.normal(1, 0.1),
            t_start=np.random.random(),
            level_start=level_start,
            level_end=level_end,
            sigma_t=sigma_t
        )
        all_messages += messages
        all_contexts += context

    return all_messages, all_contexts


if __name__ == "__main__":
    msgs, context = generate_simple_3D_reconstruction(22)
    print(msgs.messages.__str__(
        ['sender', 'receivers', 't_gen', 't_sent', 'data',
         'data_type_name']))
