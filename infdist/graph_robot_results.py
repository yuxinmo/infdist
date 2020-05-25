#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
import pickle
import sys

import plotly.graph_objs as go
import plotly.io as pio

from visualization import rate_vs_throughput_from_rate_constraint
from visualization import messageset_plotly  # NOQA: side effects: sets Orca
from optimization import simplesim


def create_constraints():
    return {
        'RATE': simplesim.create_rate_constraint_violations(
            timeslot_length=1.,
            delta=0.5,
        ),
        'TPUT': simplesim.create_throughput_constraint_violations(
            throughput=10.05,
            timeslot_length=1.,
        ),
    }


def get_latency_graph(result):
    msgs = result['received_msgs'].all()[:-5]
    print_minimum_latency(msgs)
    result = go.Scatter(
        name='latency',
        x=[m.t_rcv for m in msgs],
        y=[m.t_rcv - m.t_gen for m in msgs],
    )
    return result


def print_minimum_latency(msgs):
    latencies = [
        m.t_rcv - m.t_gen
        for m in msgs
    ]
    print(min(latencies))


def get_constraints_graphs(result, constraints):
    messages = result['received_msgs']
    return [
        go.Scatter(
            name=f'{constraint_name}',
            x=[m.t_rcv for m in messages.all()],
            y=[constraint.compute_value(messages, m.t_gen)
                for m in messages.all()],
            yaxis='y2',
            line={
                'dash': 'dash',
            },
        )
        for constraint_name, constraint in constraints.items()
    ] + [
        go.Scatter(
            name='modeled rate',
            x=[m.t_rcv for m in messages.all()],
            y=[constraints['RATE'].modeled_value(messages, m.t_gen)
                for m in messages.all()],
            yaxis='y2',
            line={
                'dash': 'dash',
            },
        )
    ]


def process_logs(logs_dir):
    robots = os.listdir(logs_dir)
    print(f'Detected logs from {len(robots)} robots: {robots}.')

    results = {}

    for robot in robots:
        with open(
            os.path.join(logs_dir, robot, 'log', 'test_results.pickle'),
            'rb',
        ) as f:
            results[robot] = pickle.load(f)

    for robot in robots:
        fig = get_robot_estimator_plot(robot, results)

        pio.write_image(
            fig,
            os.path.join('/tmp', 'robot_exps', robot + '_est.pdf'),
        )

        fig = go.Figure(
            get_robot_progress_plot(robot, results)
        )
        pio.write_image(
            fig,
            os.path.join('/tmp', 'robot_exps', robot + '_progress.pdf')
        )


def get_robot_progress_plot(robot_name, results):
    constraints = create_constraints()
    return go.Figure(
        [
            get_latency_graph(results[robot_name]),
        ] + get_constraints_graphs(results[robot_name], constraints),
        layout={
            'xaxis': {
                'title': 'time [s]',
                'domain': [0, 0.9],
            },
            'yaxis': {
                'title': 'latency [s]',
            },
            'yaxis2': {
                'title': 'constraint [Mbps]',
                'side': 'right',
                'overlaying': 'y',
            },
            'yaxis3': {
                'title': 'coef',
                'side': 'right',
                'overlaying': 'y',
                'position': 1,
            },
            'yaxis4': {
                'title': 'coef',
                'side': 'right',
                'overlaying': 'y',
                'position': 1,
            },
            'legend': {
                'orientation': 'h',
            },
        }
    )


def get_robot_estimator_plot(robot_name, results):
    result = results[robot_name]
    constraints = create_constraints()
    rate_constraint = constraints['RATE']
    tput_constraint = constraints['TPUT']
    initial_constraints = create_constraints()
    initial_rate_constraint = initial_constraints['RATE']

    return go.Figure(
        rate_vs_throughput_from_rate_constraint.graph(
            tput_constraint,
            rate_constraint,
            result['received_msgs'],
            result['train_data'],
            initial_rate_constraint,
        )
    )


if __name__ == '__main__':
    process_logs(sys.argv[1])
