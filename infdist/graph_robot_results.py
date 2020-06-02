#! /usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import csv
from collections import deque
import os
import pickle

import plotly.graph_objs as go
import plotly.io as pio

from visualization import rate_vs_throughput_from_rate_constraint
from visualization import messageset_plotly  # NOQA: side effects: sets Orca
from optimization import simplesim


SCALE = 1


def create_constraints():
    TIMESLOT_LENGTH = 5
    return {
        'RATE': simplesim.create_rate_constraint_violations(
            timeslot_length=TIMESLOT_LENGTH,
            delta=0.5,
            alpha=12,
            eta0=0.0001,  # -20
            coef_init=[[1.4]],
            scale=SCALE,  # 4
            pessymistic_latency=1.5,
        ),
        'TPUT': simplesim.create_throughput_constraint_violations(
            throughput=50.05,
            timeslot_length=TIMESLOT_LENGTH,
        ),
    }


def get_latency_graphs(result):
    start_t = 27
    end_sample = -210
    msgs = result['received_msgs'].gen_after(start_t-7)[:end_sample]
    print_minimum_latency(msgs)
    avg_lat_gen = simplesim.average_latency_generator2(3)

    return [
        go.Scatter(
            name='message latency',
            x=[m.t_gen-start_t for m in msgs],
            y=[m.t_rcv - m.t_gen for m in msgs],
            mode='markers',
            marker={
                'color': 'rgba(150, 150, 255, 1)',

            },
        ),
        go.Scatter(
            name='average latency',
            x=[m.t_gen-start_t for m in msgs],
            y=[
                avg_lat_gen.send(m)
                for m in msgs
            ],
            mode='markers',
            marker={
                'color': 'rgba(0, 0, 255, 1)',
                'size': 4.5,
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


def get_model_params_history_graphs(result):
    params_history = result['model_params_history']
    return [
        go.Scatter(
            name='coef',
            x=[t for t, a, b in params_history],
            y=[a for t, a, b in params_history],
            yaxis='y3',
            line={
                'dash': 'dash',
            },
        ),
        # go.Scatter(
        #     name='intercept',
        #     x=[t for t, a, b in params_history],
        #     y=[b for t, a, b in params_history],
        #     yaxis='y3',
        #     line={
        #         'dash': 'dash',
        #     },
        # ),
    ]


def average_generator(samples_num=10):
    def _average_generator():
        window = deque()
        avg = 0
        while True:
            d = yield avg

            window.append(d)
            while len(window) > samples_num:
                window.popleft()

            n = len(window)
            avg = sum(
                (m)*(i+1)
                for i, m in enumerate(window)
            )/((n+1) * n/2)
    g = _average_generator()
    next(g)  # start generating
    return g


def get_wireshark_graphs(wireshark_data):
    avg_gen1 = average_generator(15)  # NOQA
    avg_gen2 = average_generator(15)
    avg_gen3 = average_generator(15)
    return [
        # go.Scatter(
        #     name='RTPS metadata',
        #     x=[d['t']-10 for d in wireshark_data][100:],
        #     y=[avg_gen1.send(float(d['rtps'])/10**6*10)
        #        for d in wireshark_data][100:],
        #     yaxis='y2',
        #     line={
        #     },
        # ),
        go.Scatter(
            name='application traffic',
            x=[d['t']-10 for d in wireshark_data][100:],
            y=[avg_gen2.send(float(d['images'])/10**6*10)
               for d in wireshark_data][100:],
            yaxis='y2',
            mode='lines',
            marker={
                'size': 3,
            },
        ),
        go.Scatter(
            name='background traffic',
            x=[d['t']-10 for d in wireshark_data][100:],
            y=[avg_gen3.send(float(d['background'])/10**6*10)
               for d in wireshark_data][100:],
            yaxis='y2',
            mode='lines',
            marker={
                'size': 3,
            },
        ),
    ]


def get_wireshark_data(filename, t_start, t_end):
    result = []
    with open(filename, newline='') as csvfile:
        csvreader = csv.DictReader(csvfile)
        for row in csvreader:
            t = float(row['Interval start'])
            if t > t_start and t < t_end:
                row['t'] = t - t_start
                result.append(row)
    return result


def gen_publication_plots(
    logs_dir,
    wireshark_data_file,
    robot_name,
):
    robots = [robot_name]
    results = {}
    wireshark_data = get_wireshark_data(
        wireshark_data_file,
        90, 200,
    )

    for robot in robots:
        with open(
            os.path.join(logs_dir, robot, 'log', 'test_results.pickle'),
            'rb',
        ) as f:
            results[robot] = pickle.load(f)

    for robot in robots:
        fig = get_robot_estimator_plot(robot, results, True)

        fig.update_layout(
            width=400,
            height=300,
            margin=go.layout.Margin(
                l=0,  # NOQA
                r=0,
                b=0,
                t=0,
                pad=0
            ),
            template='plotly_white',
            legend={
                'orientation': 'h',
                'x': 0,
                'y': 1.2,
            },
        )

        pio.write_image(
            fig,
            os.path.join(
                "/home/zeroos/uni/papers/2020-MBarcis-infdist_adaptation/data",
                "robot_estimator.pdf",
            )
        )

        fig = go.Figure(
            get_robot_progress_plot(
                robot,
                results,
                wireshark_data,
            )
        )

        fig.update_xaxes(range=[0, 100])
        fig.update_layout(
            width=400,
            height=300,
            margin=go.layout.Margin(
                l=0,  # NOQA
                r=0,
                b=0,
                t=0,
                pad=0
            ),
            template='plotly_white',
            legend={
                'orientation': 'h',
                'x': 0,
                'y': 1.2,
            },
        )
        pio.write_image(
            fig,
            os.path.join(
                "/home/zeroos/uni/papers/2020-MBarcis-infdist_adaptation/data",
                "background_traffic_experiment.pdf",
            )
        )


def process_logs(logs_dir, wireshark_data_file, robot_name):
    if robot_name == 'all':
        robots = os.listdir(logs_dir)
    else:
        robots = [robot_name]
    print(f'Detected logs from {len(robots)} robots: {robots}.')

    results = {}
    wireshark_data = get_wireshark_data(
        wireshark_data_file,
        90, 200,
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
    constraints = create_constraints()  # NOQA
    return go.Figure(
        (
            get_latency_graphs(results[robot_name])
            # + get_constraints_graphs(results[robot_name], constraints)
            + get_wireshark_graphs(wireshark_data)
            # + get_model_params_history_graphs(results[robot_name])
        ),
        layout={
            'xaxis': {
                'title': 'time [s]',
                'domain': [0, 0.9],
            },
            'yaxis': {
                'title': 'latency [s]',
                'range': [0, 6],
            },
            'yaxis2': {
                'title': 'reception rate [Mbps]',
                'side': 'right',
                'overlaying': 'y',
                'range': [0, 15]
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


def get_robot_estimator_plot(robot_name, results, publication_version=False):
    result = results[robot_name]
    constraints = create_constraints()
    rate_constraint = constraints['RATE']
    tput_constraint = constraints['TPUT']
    initial_constraints = create_constraints()
    initial_rate_constraint = initial_constraints['RATE']

    print(robot_name)
    print(result.keys())
    print(len(result['train_data']))

    if publication_version:
        f = rate_vs_throughput_from_rate_constraint.graph_pub
    else:
        f = rate_vs_throughput_from_rate_constraint.graph

    graphs = f(
        tput_constraint,
        rate_constraint,
        result['received_msgs'],
        result['train_data'],
        initial_rate_constraint,
        {
            '100%': result['model_params_history'][-1][1:],
            '5%': result['model_params_history'][
                int(len(result['model_params_history'])*0.05)
            ][1:],
            '25%': result['model_params_history'][
                int(len(result['model_params_history'])*0.25)
            ][1:],
            '10%': result['model_params_history'][
                int(len(result['model_params_history'])*0.10)
            ][1:],
        },
        scale=SCALE,
    )
    return go.Figure(
        graphs,
        layout={
            'xaxis': {
                'title': 'publishing rate [Mbps]',
                'range': [0, 10],
            },
            'yaxis': {
                'title': r'$\overline{|{W}|_\Delta} \mathrm{\,[Mbps]}$',
                'range': [0, 20],
            },
        }
    )


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate plots from results')
    parser.add_argument('--publication',
                        action='store_const',
                        const=True, default=False,
                        help='generate publication version')
    parser.add_argument('results_dir', type=str,
                        help='results directory')
    parser.add_argument('wireshark_data', type=str,
                        help='csv file with wireshark results')
    parser.add_argument('robot_name', type=str,
                        help='name of a robot to process, '
                             '"all" processes all robots')
    args = parser.parse_args()
    if args.publication:
        gen_publication_plots(
            args.results_dir, args.wireshark_data, args.robot_name
        )
    else:
        process_logs(
            args.results_dir, args.wireshark_data, args.robot_name
        )
