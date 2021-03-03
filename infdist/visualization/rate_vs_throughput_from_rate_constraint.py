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
    scale=12,
    graph_model_development=True,
):
    if model_params is None:
        model_params = {}
    if graph_model_development:
        i = 0
        for throughput, rate in train_data:
            i += 1
            X = [[throughput*scale]]
            Y = [rate*scale]
            rate_constraint.rate_model.partial_fit(X, Y)
            # print(X, Y, ' || ',
            #       rate_constraint.rate_model.coef_[0],
            #       rate_constraint.rate_model.intercept_[0])
            if i % 1000 == 0:
                model_params[f'{i/len(train_data)}'] = (
                    rate_constraint.rate_model.coef_[0],
                    rate_constraint.rate_model.intercept_[0],
                )

    basic_plots = [
        # go.Scatter(
        #     name='data',
        #     x=[
        #         tput_constraint.compute_value(messages, m.t_gen)
        #         for m in messages.all()
        #     ],
        #     y=[
        #         rate_constraint.compute_value(messages, m.t_gen)
        #         for m in messages.all()
        #     ],
        #     mode='markers',
        #     marker={
        #         'opacity': 0.02,
        #         'color': 'gray',
        #     },
        # ),
        go.Scatter(
            name='first half of train data',
            x=[
                x
                for x, y in train_data[:len(train_data)//2]
            ],
            y=[
                y
                for x, y in train_data[:len(train_data)//2]
            ],
            mode='markers',
            marker={
                'opacity': 0.02,
            },
        ),
        go.Scatter(
            name='second half of train data',
            x=[
                x
                for x, y in train_data[len(train_data)//2:]
            ],
            y=[
                y
                for x, y in train_data[len(train_data)//2:]
            ],
            mode='markers',
            marker={
                'opacity': 0.02,
            },
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
        # go.Scatter(
        #     name='initial model',
        #     x=[
        #         tput_constraint.compute_value(messages, m.t_gen)
        #         for m in messages.all()
        #     ],
        #     y=[
        #         initial_rate_constraint.modeled_value(messages, m.t_gen)
        #         for m in messages.all()
        #     ],
        #     line={
        #         'color': 'rgba(0,0,0,0.5)',
        #     },
        # ),
    ]
    progress_plots = []
    for progress, params in model_params.items():
        rate_constraint.rate_model.coef_[0] = params[0]
        rate_constraint.rate_model.intercept_[0] = params[1]

        progress_plots.append(go.Scatter(
            name=f'model at {float(progress)*100:.2f}%',
            x=[
                tput_constraint.compute_value(messages, m.t_gen)
                for m in messages.all()
            ],
            y=[
                rate_constraint.modeled_value(messages, m.t_gen)
                for m in messages.all()
            ],
            line={
                'color': f'rgba({1-float(progress)}, {float(progress)}, 0, 0.2)',
            }
        ))
    return basic_plots + progress_plots


def graph_pub(
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

    congestion_split = len(train_data)//7

    basic_plots = [
        go.Scatter(
            name='train data',
            x=[
                x
                for x, y in train_data[congestion_split*3:congestion_split*4]
            ],
            y=[
                y
                for x, y in train_data[congestion_split*3:congestion_split*4]
            ],
            mode='markers',
            marker={
                'opacity': 0.5,
            },
        ),
        go.Scatter(
            name='train data',
            x=[
                x
                for x, y in train_data[:congestion_split]
            ],
            y=[
                y
                for x, y in train_data[:congestion_split]
            ],
            mode='markers',
            marker={
                'opacity': 0.5,
            },
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
