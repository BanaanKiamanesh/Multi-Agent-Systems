"""Discrete-Time Consensus: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


def WeightedLaplacian(Graph):
    """Return receiver-row Laplacian and sorted nodes; L @ X is local disagreement."""
    Nodes = list(sorted(Graph.nodes()))
    Weights = nx.to_numpy_array(Graph, nodelist=Nodes, weight="weight", dtype=float)
    if Graph.is_directed():
        Weights = Weights.T
    if not np.all(np.isfinite(Weights)) or np.any(Weights < 0):
        raise ValueError("Communication weights must be finite and nonnegative")
    return np.diag(np.sum(Weights, axis=1)) - Weights, Nodes


def SimulateDiscreteConsensus(Graph, InitialState, StepSize=0.2, StepNum=100):
    """x[k+1]=(I-hL)x[k]; enforce nonnegative stochastic mixing weights."""
    Laplacian, Nodes = WeightedLaplacian(Graph)
    if StepNum < 1 or int(StepNum) != StepNum or StepSize <= 0:
        raise ValueError("Positive StepSize and an integer StepNum are required")
    if StepSize * np.max(np.diag(Laplacian)) >= 1:
        raise ValueError("Use StepSize < 1 / maximum weighted in-degree")
    InitialState = np.asarray(InitialState, dtype=float)
    if InitialState.shape != (len(Nodes),):
        raise ValueError("InitialState must contain one scalar per node")
    StepNum = int(StepNum)
    Mixing = np.eye(len(Nodes)) - StepSize * Laplacian
    Motion = np.empty((StepNum + 1, len(Nodes)))
    Motion[0] = InitialState
    for Index in range(StepNum):
        Motion[Index + 1] = Mixing @ Motion[Index]
    return np.arange(StepNum + 1, dtype=float), Motion, Mixing
