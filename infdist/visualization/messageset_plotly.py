import plotly.graph_objs as go
import plotly
import plotly.io as pio
import numpy as np
import os
from pathlib import Path


LOCAL_ORCA_PATH = os.path.join(
    str(Path.home()), 'node_modules', '.bin', 'orca'
)

if os.path.exists("/orca.sh"):
    plotly.io.orca.config.executable = '/orca.sh'
elif os.path.exists(LOCAL_ORCA_PATH):
    plotly.io.orca.config.executable = LOCAL_ORCA_PATH


_PLOTLY_COLORS = [
    (int('1f', 16), int('77', 16), int('b4', 16)),  # muted blue
    (int('ff', 16), int('7f', 16), int('0e', 16)),  # safety orange
    (int('2c', 16), int('a0', 16), int('2c', 16)),  # cooked asparagus green
    (int('d6', 16), int('27', 16), int('28', 16)),  # brick red
    (int('94', 16), int('67', 16), int('bd', 16)),  # muted purple
    (int('8c', 16), int('56', 16), int('4b', 16)),  # chestnut brown
    (int('e3', 16), int('77', 16), int('c2', 16)),  # raspberry yogurt pink
    (int('7f', 16), int('7f', 16), int('7f', 16)),  # middle gray
    (int('bc', 16), int('bd', 16), int('22', 16)),  # curry yellow-green
    (int('17', 16), int('be', 16), int('cf', 16))   # blue-teal
]


def write_messageset_plots(messages, context, f):
    for msg_type in context.message_types:
        type_messages = messages.filter(data_type=msg_type.data_type_name)
        plots = [gen_message_plots(
            type_messages,
            msg_type.utility_cls(),
        )]
        write_plots(plots, f + msg_type.data_type_name + '.pdf')


def get_plotly_color(i, alpha=1):
    return 'rgba({}, {}, {}, {})'.format(*(_PLOTLY_COLORS[i] + (alpha,)))


def EMPTY_HISTORY(_):
    return {}


def gen_message_plots(messages, utility, history=EMPTY_HISTORY,
                      message_annotations=[], dash='dash'):
    data = []
    annotations = []

    # arrows
    annotations += [
        dict(
            x=m.t_sent,
            y=1,
            text=' '.format(i),
            showarrow=True,
            ay=20,
            ax=0,
        ) for i, m in enumerate(messages.all())
    ] + [
        dict(
            x=m.t_rcv,
            y=1,
            text=(
                message_annotations[i] if len(message_annotations) > i
                else ' '
            ).format(i),
            showarrow=True,
            ay=-20,
            ax=0,
        ) for i, m in enumerate(messages.all())
    ]

    # legend for messages
    d1 = go.Scatter(
        x=(0, .01),
        y=(0, .01),
        name='Message utilities',
        mode='lines',
        line={
            'width': 3,
            'dash': dash,
            'color': 'black',
        }
    )
    data.append(d1)

    # messages
    for i, m in enumerate(messages.all()):
        xs = np.arange(
            m.t_rcv, messages.t_end, 0.01
        )
        ys = tuple(utility(m, x, history(x)) for x in xs)

        d1 = go.Scatter(
            x=xs,
            y=ys,
            name='Message {}'.format(i),
            showlegend=False,
            line={
                'width': 3,
                'dash': dash,
            }
        )
        data.append(d1)
    return {
        'data': data,
        'annotations': annotations,
        'xaxis': {
            'range': [0, messages.t_end],
            'title': 'time [s]',
        },
        'yaxis': {
            'rangemode': 'tozero',
            'title': 'utility',
        }
    }


def gen_aggregation_plots(messages, utility, AggregationCls,
                          history=EMPTY_HISTORY, precision=0.0001):
    data = []

    aggregation = AggregationCls(utility, messages)

    xs = np.arange(
        0, messages.t_end, precision
    )
    ys = aggregation.evaluate(xs, [history(x) for x in xs])

    d1 = go.Scatter(
        name='Aggregation',
        x=xs,
        y=ys,
        line={
            'width': 1,
            'dash': 'solid',
            'color': '#aaaaff',
        },
        fill='tozeroy',
    )
    data.append(d1)

    return {'data': data}


def gen_data_plot(plot_data, name=None, line_args=None):
    data = []

    xs, ys = zip(*plot_data)

    if line_args is None:
        line_args = {
            'width': 1,
            'dash': 'dot',
            'color': '#000000',
        }

    d1 = go.Scatter(
        name=name or 'data',
        x=xs,
        y=ys,
        line=line_args,
        mode='lines',
    )

    data.append(d1)
    return {'data': data}


def _generate_fig(plots, additional_layout_kwargs={}, width=400, height=200):
    data = sum([p.get('data', []) for p in plots], [])
    layout_kwargs = {
        "margin": go.layout.Margin(
                l=40,  # NOQA
                r=0,
                b=25,
                t=10,
                pad=0
            ),
        "width": width,
        "height": height,
        "legend": {
            "orientation": "h",
            "bgcolor": "rgba(0, 0, 0, 0)",
            "x": 0,
            "y": 1.1,
        }
    }
    layout_kwargs.update(additional_layout_kwargs)
    for d in plots:
        for k, v in d.items():
            if k != 'data':
                if k not in layout_kwargs:
                    layout_kwargs[k] = []
                if isinstance(v, dict):
                    layout_kwargs[k] = v
                else:
                    layout_kwargs[k] += v
    layout = go.Layout(
        **layout_kwargs
    )
    return go.Figure(data, layout=layout)


def display_plots(plots, width=400, height=200):
    fig = _generate_fig(plots, width=width, height=height)
    plotly.offline.iplot(fig)


def write_plots(plots, f="/tmp/plot.pdf", width=800, height=300):
    if not f:
        return
    fig = _generate_fig(plots, width=width, height=height)
    pio.write_image(fig, f)


def plot(messages, utility, aggregation, history=lambda _: {}):
    messages_plots = gen_message_plots(messages, utility, history)
    aggregation_plots = gen_aggregation_plots(messages, utility, aggregation,
                                              history)
    display_plots([messages_plots, aggregation_plots])
