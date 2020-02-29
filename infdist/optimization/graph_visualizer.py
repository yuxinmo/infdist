from graphviz import Digraph, nohtml
from .tree_node_wrapper import TreeNodeWrapper


def node_formatter(node):
    return '{ ' + str(TreeNodeWrapper(node)).replace('\n', '|') + ' }'


def show_graph(root):
    dot = Digraph(node_attr={'shape': 'record'}, format='pdf')
    nodes_queue = [(root, None)]
    node_i = 0

    while nodes_queue:
        node_ident = str(node_i)
        node, parent_ident = nodes_queue.pop()
        # dot.node(node_ident, nohtml(str(node)))
        dot.node(node_ident, nohtml(
            node_formatter(node)
        ))
        if parent_ident is not None:
            dot.edge(parent_ident, node_ident)

        for child in node.children:
            nodes_queue.append((child, node_ident))

        node_i += 1
    dot.render('/tmp/graphviz_plot.gz', view=True)
