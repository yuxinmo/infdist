import numpy as np

from .aggregations import AggregationMostRecent
from .models import MissionContext, MessageSet, Message, InformationType
from .utilities import UtilityBattery, UtilityPosition
from .message_forecast import FullKnowledgeTypeForecast, PeriodicTypeForecast  # NOQA

BATTERY_DATA_TYPE = 'batt'
POSITION_DATA_TYPE = 'position'


def generate_periodic_messages(
    t_end, sender, receivers, data_type_name,
    t_start=0, f=1, data_f=lambda t: {}, sigma_t=0,
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
            abs(np.random.normal(t, sigma_t)),
            gen_data_type_name(sender),
            data_f(t)
        )
        for t in np.arange(t_start, t_end, 1/f)
    ], t_start)


def generate_batt_messages(t_end, sender, receivers, t_start=0, f=1,
                           level_start=1, level_end=0, sigma_t=0,
                           max_depl_rate_mi=0.003, max_depl_rate_sigma=0.002
                           ):

    mi = np.random.normal(0.2, 0.01)

    def batt_level(t):
        a = (level_start - level_end) / (t_start - t_end)
        b = level_end - a * t_end
        level = a*t+b
        return {
            'battery_level': level,
            'max_depl_rate': max(
                0.003, np.random.normal(mi, 0.002)
            )
        }

    messages = generate_periodic_messages(
        t_end, sender, receivers,
        BATTERY_DATA_TYPE, t_start, f, batt_level,
        sigma_t,
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
                    weight=np.random.random()*10+0.5,
                )
                for data_type_name in set(
                    m.data_type_name
                    for m in messages.all()
                )

            ])
        )
    )


def generate_pos_messages(t_end, sender, receivers, t_start=0, f=5,
                          sigma_t=0):
    messages = generate_periodic_messages(
        t_end, sender, receivers, POSITION_DATA_TYPE, t_start, f,
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
            t_end, sender, receivers,
            f=np.random.normal(1, 0.1),
            t_start=int(np.random.random()*100)/100,
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
