from graphviz import Digraph, nohtml
from .tree_node_wrapper import TreeNodeWrapper


def node_formatter(node, additional_text=''):
    return '{ ' + (str(node)+additional_text).replace('\n', '|') + ' }'


text_color = 'grey90'
borders_color = 'grey90'
arrows_color = 'grey90'

NORMAL_NODE_ATTR = {
    'shape': 'record',
    'style': 'filled',
    'fillcolor': 'white',
    'gradientangle': '270',
    'color': borders_color,
    'fontcolor': text_color,
}


def show_graph(
    root,
    emphasized_message=None,
    f='/tmp/graphviz',
    sim_num=1500,
):
    dot = Digraph(
        node_attr=NORMAL_NODE_ATTR,
        format='svg',
    )
    dot.attr(
        bgcolor='transparent',
    )
    nodes_queue = [(root, None, 0, 0)]
    node_i = 0

    while nodes_queue:
        node_ident = str(node_i)
        node, parent_ident, child_num, level = nodes_queue.pop()
        wrapped_node = TreeNodeWrapper(node)

        color_perc = (wrapped_node.node.visits/sim_num)**0.3
        start_color = 'black'
        if wrapped_node.message == emphasized_message:
            start_color = '0, 0.8, 0.8'

        dot.attr('node', dict(
            NORMAL_NODE_ATTR,
            **{'fillcolor': f'{start_color}:0.66, 0.8, {color_perc}'}
        ))

        dot.node(node_ident, nohtml(
            node_formatter(wrapped_node)
        ))
        if parent_ident is not None:
            dot.edge(
                parent_ident,
                node_ident,
                label='send' if child_num == 1 else 'drop',
                color=arrows_color,
                fontcolor=arrows_color,
            )

        for i, child in enumerate(node.children):
            nodes_queue.append((child, node_ident, i, level+1))

        node_i += 1
    dot.render(f, view=False)
