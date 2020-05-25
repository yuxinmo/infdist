#! /usr/bin/env python
# -*- coding: utf-8 -*-
import plotly.graph_objs as go


def graph(
    tput_constraint,
    rate_constraint,
    messages,
    train_data,
    initial_rate_constraint,
):
    return [
        go.Scatter(
            name='data',
            x=[
                tput_constraint.compute_value(messages, m.t_gen)
                for m in messages.all()
            ],
            y=[
                rate_constraint.compute_value(messages, m.t_gen)
                for m in messages.all()
            ],
            mode='markers',
        ),
        go.Scatter(
            name='train data',
            x=[
                x
                for x, y in train_data
            ],
            y=[
                y
                for x, y in train_data
            ],
            mode='markers',
        ),
        go.Scatter(
            name='final model',
            x=[
                tput_constraint.compute_value(messages, m.t_gen)
                for m in messages.all()
            ],
            y=[
                rate_constraint.modeled_value(messages, m.t_gen)
                for m in messages.all()
            ],
        ),
        go.Scatter(
            name='initial model',
            x=[
                tput_constraint.compute_value(messages, m.t_gen)
                for m in messages.all()
            ],
            y=[
                initial_rate_constraint.modeled_value(messages, m.t_gen)
                for m in messages.all()
            ],
        ),
    ]
