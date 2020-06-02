#! /usr/bin/env python
# -*- coding: utf-8 -*-

import csv
import os
import pickle
import sys

import plotly.graph_objs as go
import plotly.io as pio

from visualization import rate_vs_throughput_from_rate_constraint
from visualization import messageset_plotly  # NOQA: side effects: sets Orca
from optimization import simplesim


SCALE = 1


def create_constraints():
    return {
        'RATE': simplesim.create_rate_constraint_violations(
            timeslot_length=1.,
            delta=0.5,
            alpha=7,
            eta0=1e-3,  # -20
            coef_init=[[2]],
            scale=SCALE,  # 4
        ),
        'TPUT': simplesim.create_throughput_constraint_violations(
            throughput=50.05,
            timeslot_length=1.,
        ),
    }


def get_latency_graphs(result):
    msgs = result['received_msgs'].all()[:-5]
    print_minimum_latency(msgs)
    avg_lat_gen = simplesim.average_latency_generator2(1)

    return [
        go.Scatter(
            name='message latency',
            x=[m.t_gen for m in msgs],
            y=[m.t_rcv - m.t_gen for m in msgs],
            mode='markers',
            marker_color='rgba(150, 150, 255, 1)'
        ),
        go.Scatter(
            name='average latency',
            x=[m.t_gen for m in msgs],
            y=[
                avg_lat_gen.send(m)
                for m in msgs
            ],
            line={
                'color': 'blue',
            },
        )
    ]


def print_minimum_latency(msgs):
    latencies = [
        m.t_rcv - m.t_gen
        for m in msgs
    ]
    if latencies:
        print(min(latencies))
    else:
        print("no data")


def get_constraints_graphs(result, constraints):
    messages = result['received_msgs']
    return [
        go.Scatter(
            name=f'{constraint_name}',
            x=[m.t_gen for m in messages.all()],
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
            x=[m.t_gen for m in messages.all()],
            y=[constraints['RATE'].modeled_value(messages, m.t_gen)
                for m in messages.all()],
            yaxis='y2',
            line={
                'dash': 'dash',
            },
        )
    ]


def get_wireshark_graphs(wireshark_data):
    return [
        go.Scatter(
            name='RTPS metadata',
            x=[d['t'] for d in wireshark_data],
            y=[float(d['rtps'])/10**6 for d in wireshark_data],
            yaxis='y2',
            line={
            },
        ),
        go.Scatter(
            name='RTPS payload',
            x=[d['t'] for d in wireshark_data],
            y=[float(d['images'])/10**6 for d in wireshark_data],
            yaxis='y2',
            line={
            },
        ),
        go.Scatter(
            name='background traffic',
            x=[d['t'] for d in wireshark_data],
            y=[float(d['background'])/10**6 for d in wireshark_data],
            yaxis='y2',
            line={
            },
        ),
    ]


def get_wireshark_data(filename, t_start, t_end):
    result = []
    with open(filename, newline='') as csvfile:
        csvreader = csv.DictReader(csvfile)
        for row in csvreader:
            t = int(row['Interval start'])
            if t > t_start and t < t_end:
                row['t'] = t - t_start
                result.append(row)
    return result


def process_logs(logs_dir):
    robots = os.listdir(logs_dir)
    print(f'Detected logs from {len(robots)} robots: {robots}.')

    results = {}
    wireshark_data = get_wireshark_data(
        "/home/zeroos/wireshark_data.csv",
        540, 600,
    )

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
            get_robot_progress_plot(
                robot,
                results,
                wireshark_data,
            )
        )
        pio.write_image(
            fig,
            os.path.join('/tmp', 'robot_exps', robot + '_progress.pdf')
        )


def get_robot_progress_plot(robot_name, results, wireshark_data):
    constraints = create_constraints()
    return go.Figure(
        (
            get_latency_graphs(results[robot_name])
            + get_constraints_graphs(results[robot_name], constraints)
            + get_wireshark_graphs(wireshark_data)
        ),
        layout={
            'xaxis': {
                'title': 'time [s]',
                'domain': [0, 0.9],
            },
            'yaxis': {
                'title': 'latency [s]',
            },
            'yaxis2': {
                'title': 'reception rate [Mbps]',
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

    print(robot_name)
    print(result.keys())
    print(len(result['train_data']))
    return go.Figure(
        rate_vs_throughput_from_rate_constraint.graph(
            tput_constraint,
            rate_constraint,
            result['received_msgs'],
            result['train_data'],
            initial_rate_constraint,
            {
                '100%': result['model_params_history'][-1],
                '75%': result['model_params_history'][
                    int(len(result['model_params_history'])*0.75)
                ],
            },
            scale=SCALE,
        )
    )


if __name__ == '__main__':
    process_logs(sys.argv[1])
