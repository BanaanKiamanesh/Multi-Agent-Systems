"""Metropolis Canal Consensus: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


def MetropolisWeights(Graph):
    """Symmetric stochastic weights 1/(1+max(deg_i,deg_j))."""
    if Graph.is_directed() or not nx.is_connected(Graph):
        raise ValueError("Use a connected undirected graph")
    Nodes = sorted(Graph.nodes())
    Matrix = np.zeros((len(Nodes), len(Nodes)))
    Index = {Node: Number for Number, Node in enumerate(Nodes)}
    for One, Two in Graph.edges():
        if One == Two:
            raise ValueError("Self-loops are unsupported")
        Weight = 1.0 / (1 + max(Graph.degree(One), Graph.degree(Two)))
        Matrix[Index[One], Index[Two]] = Matrix[Index[Two], Index[One]] = Weight
    np.fill_diagonal(Matrix, 1 - Matrix.sum(axis=1))
    return Matrix
