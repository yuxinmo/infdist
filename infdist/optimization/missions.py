import numpy as np

from .aggregations import AggregationMostRecent
from .models import MissionContext, MessageSet, Message, InformationType
from .utilities import UtilityBattery, UtilityPosition
from .message_forecast import FullKnowledgeTypeForecast, PeriodicTypeForecast  # NOQA

BATTERY_DATA_TYPE = 'batt'
POSITION_DATA_TYPE = 'position'

presets = [
    {
        'max_depl_rate_mi': lambda: np.random.normal(0.2, 0.2),
        'max_depl_rate': lambda mi: max(0.003, np.random.normal(mi, 0.001)),
        't_gen': lambda t: abs(np.random.normal(t, 0.1)),
        'topic_weight': lambda i: 3**(i+1),
        # 'message_size': lambda i: 2048 - i*25,
        'message_size': lambda i: 2048,
    },
    {
        'max_depl_rate_mi': lambda: np.random.normal(0.3, 0.01),
        'max_depl_rate': lambda mi: max(0.003, np.random.normal(mi, 0.001)),
        't_gen': lambda t: abs(np.random.normal(t, 0.001)),
        'topic_weight': lambda i: i*1000+1
    },
    {
        'max_depl_rate_mi': lambda: 0.7,
        'max_depl_rate': lambda mi: mi,
        't_gen': lambda t: t,
        'topic_weight': lambda i: i**(0.1)+1,
    },
]


def generate_periodic_messages(
    t_end, sender, receivers, data_type_name,
    t_start=0, f=1, data_f=lambda t: {},
    msgset=0,
    append_sender_to_data_type_name=False,
):
    def gen_data_type_name(sender):
        if append_sender_to_data_type_name:
            return data_type_name + str(sender)
        else:
            return data_type_name

    return MessageSet(t_end, [
        Message(
            sender,
            set(receivers) - set([sender]),
            presets[msgset]['t_gen'](t),
            gen_data_type_name(sender),
            presets[msgset]['message_size'](sender),
            data_f(t)
        )
        for t in np.arange(t_start, t_end, 1/f)
    ], t_start)


def generate_batt_messages(t_end, sender, receivers, t_start=0, f=1,
                           level_start=1, level_end=0,
                           msgset=0,
                           ):

    mi = presets[msgset]['max_depl_rate_mi']()

    def batt_level(t):
        a = (level_start - level_end) / (t_start - t_end)
        b = level_end - a * t_end
        level = a*t+b
        return {
            'battery_level': level,
            'max_depl_rate': presets[msgset]['max_depl_rate'](mi),
        }

    messages = generate_periodic_messages(
        t_end, sender, receivers,
        BATTERY_DATA_TYPE, t_start, f, batt_level,
        msgset=msgset,
        append_sender_to_data_type_name=True,
    )

    full_knowledge = False
    if full_knowledge:
        forecast_cls = FullKnowledgeTypeForecast

        def forecast_kwargs(data_type_name):
            return {
                'data_type_name': data_type_name,
                'messages': messages,
            }
    else:
        forecast_cls = PeriodicTypeForecast

        def forecast_kwargs(data_type_name):
            return {
                'data_type_name': data_type_name,
                't_end': t_end,
                'max_depl_rate': mi,
                'battery_level': level_start,
                'receivers': receivers,
                'sender': sender,
                'initial_t_start': t_start,
                'T': 1/f,
                'size': presets[msgset]['message_size'](sender),
            }

    return (
        messages,
        MissionContext(
            set([
                InformationType(
                    data_type_name,
                    utility_cls=UtilityBattery,
                    aggregation_cls=AggregationMostRecent,
                    message_forecast_cls=forecast_cls,
                    message_forecast_kwargs=forecast_kwargs(data_type_name),
                    weight=presets[msgset]['topic_weight'](sender),
                )
                for data_type_name in set(
                    m.data_type_name
                    for m in messages.all()
                )
            ])
        )
    )


def generate_pos_messages(t_end, sender, receivers, t_start=0, f=5):
    messages = generate_periodic_messages(
        t_end, sender, receivers, POSITION_DATA_TYPE, t_start, f,
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
    t_end, msgset=0, senders={0, 1}, receivers=None, seed=0,
):
    if receivers is None:
        receivers = senders

    np.random.seed(seed)

    all_messages = MessageSet(t_end, [])
    all_contexts = MissionContext(set())

    for sender in senders:
        level_start = 1
        level_end = level_start
        # level_end = level_start * np.random.random()
        messages, context = generate_batt_messages(
            t_end, sender, receivers,
            f=np.random.normal(3, 0.1),
            t_start=int(np.random.random()) + sender*30,
            level_start=level_start,
            level_end=level_end,
            msgset=msgset,
        )
        all_messages += messages
        all_contexts += context

    return all_messages, all_contexts


if __name__ == "__main__":
    msgs, context = generate_simple_3D_reconstruction(22)
    print(msgs.messages.__str__(
        ['sender', 'receivers', 't_gen', 't_sent', 'data',
         'data_type_name']))
