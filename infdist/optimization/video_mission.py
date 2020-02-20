#! /usr/bin/env python

"""
This file is only for TESTING purposes of hardware-based experiments.
"""

from optimization.models import (
    InformationType,
    MessageSet,
    MissionContext,
)
from optimization.aggregations import AggregationMostRecent
from optimization.utilities import UtilityBattery
from optimization.message_forecast import PeriodicTypeForecast
from optimization.missions import generate_periodic_messages


def generate_mission_context(agents, data_types, t_end, T):
    return MissionContext(
        set([
            InformationType(
                f'{data_type}{agent}',
                utility_cls=UtilityBattery,
                aggregation_cls=AggregationMostRecent,
                message_forecast_cls=PeriodicTypeForecast,
                message_forecast_kwargs={
                    't_end': t_end,
                    'data_type_name': f'{data_type}{agent}',
                    'max_depl_rate': 0.1,
                    'battery_level': 1,
                    'receivers': ['base_station'],
                    'T': T,
                    'sender': agent,
                },
                weight=1
            )
            for data_type in data_types
            for agent in agents
        ])
    )


def generate_example_video_mission(
    agents, t_end,
):
    T = 0.2
    data_type_name = "video"
    ctx = generate_mission_context(agents, [data_type_name], t_end, T)
    msgs = MessageSet(t_end, [])
    for sender in agents:
        msgs += generate_periodic_messages(
            t_end, sender, agents, data_type_name,
            f=1/T,
            data_f=lambda t: {'battery_level': 1, 'max_depl_rate': 0.1},
            append_sender_to_data_type_name=True,
        )
    return msgs, ctx
