"""Max-Min Message Consensus: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


def MaxMinConsensus(Graph, Initial):
    """Synchronous neighbor extrema propagation; a discrete counterpart to Cortes' extrema flows."""
    if Graph.is_directed() or not nx.is_connected(Graph):
        raise ValueError("Use a connected undirected graph")
    Nodes = sorted(Graph.nodes())
    Initial = np.asarray(Initial, dtype=float)
    if Initial.shape != (len(Nodes),):
        raise ValueError("Provide one initial scalar per node")
    Adjacency = nx.to_numpy_array(Graph, nodelist=Nodes, weight=None).astype(bool)
    np.fill_diagonal(Adjacency, True)
    State = np.column_stack([Initial, Initial])
    Motion = [State.copy()]
    for _ in range(nx.diameter(Graph)):
        High = np.max(np.where(Adjacency, State[None, :, 0], -np.inf), axis=1)
        Low = np.min(np.where(Adjacency, State[None, :, 1], np.inf), axis=1)
        State = np.column_stack([High, Low])
        Motion.append(State.copy())
    return np.arange(len(Motion), dtype=float), np.array(Motion)
