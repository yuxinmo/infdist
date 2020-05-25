#! /usr/bin/env python
# -*- coding: utf-8 -*-
import plotly.graph_objs as go


def graph(
    tput_constraint,
    rate_constraint,
    messages,
    train_data,
    initial_rate_constraint,
    model_params=None,
    scale=1,
):
    if model_params is None:
        model_params = {}
    i = 0
    for throughput, rate in train_data:
        i += 1
        X = [[throughput*scale]]
        Y = [rate*scale]
        # print(X, Y)
        rate_constraint.rate_model.partial_fit(X, Y)
        if i % 20 == 0:
            model_params[f'{i}'] = (
                rate_constraint.rate_model.coef_[0],
                rate_constraint.rate_model.intercept_[0],
            )

    basic_plots = [
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
    progress_plots = []
    for progress, params in model_params.items():
        rate_constraint.rate_model.coef_[0] = params[0]
        rate_constraint.rate_model.intercept_[0] = params[1]

        progress_plots.append(go.Scatter(
            name=f'model at {progress}',
            x=[
                tput_constraint.compute_value(messages, m.t_gen)
                for m in messages.all()
            ],
            y=[
                rate_constraint.modeled_value(messages, m.t_gen)
                for m in messages.all()
            ],
        ))
    return basic_plots + progress_plots
