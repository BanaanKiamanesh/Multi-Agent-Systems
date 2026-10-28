"""Delayed Consensus: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


def ConnectedGraph(AgentNum=6):
    Graph = nx.cycle_graph(AgentNum)
    Graph.add_edges_from((Index, Index + 2) for Index in range(0, AgentNum - 2, 2))
    nx.set_edge_attributes(Graph, 1.0, "weight")
    return Graph


def WeightedLaplacian(Graph):
    """Return receiver-row Laplacian and sorted nodes; L @ X is local disagreement."""
    Nodes = list(sorted(Graph.nodes()))
    Weights = nx.to_numpy_array(Graph, nodelist=Nodes, weight="weight", dtype=float)
    if Graph.is_directed():
        Weights = Weights.T
    if not np.all(np.isfinite(Weights)) or np.any(Weights < 0):
        raise ValueError("Communication weights must be finite and nonnegative")
    return np.diag(np.sum(Weights, axis=1)) - Weights, Nodes


def NormalizedLaplacian(Graph):
    """Return D^-1 L, with zero rows for isolated nodes.

    Even for an undirected graph this row normalization generally preserves
    a degree-weighted average, rather than the arithmetic mean.
    """
    Laplacian, Nodes = WeightedLaplacian(Graph)
    Degree = np.diag(Laplacian)
    Scale = np.where(Degree > 0, Degree, 1.0)
    return Laplacian / Scale[:, None], Nodes


def TimeGrid(SimTime, Dt):
    """Uniform samples ending exactly at SimTime, with step no larger than Dt."""
    if not np.isfinite(SimTime) or not np.isfinite(Dt) or min(SimTime, Dt) <= 0:
        raise ValueError("SimTime and Dt must be finite and positive")
    return np.linspace(0.0, SimTime, int(np.ceil(SimTime / Dt)) + 1)


def SimulateDelay(Laplacian, InitialState, Delay=0.3, Gain=0.9, SimTime=16, Dt=0.01):
    """Forward-Euler method of steps; constant history X(t <= 0) = InitialState.

    Delay is rounded to the nearest multiple of the actual integration step.
    """
    Time = TimeGrid(SimTime, Dt)
    Dt = Time[1] - Time[0]
    InitialState = np.asarray(InitialState, dtype=float)
    Laplacian = np.asarray(Laplacian, dtype=float)
    if not np.isfinite(Delay) or Delay < 0 or not np.isfinite(Gain) or Gain <= 0:
        raise ValueError("Delay must be finite and nonnegative; Gain must be positive")
    if not np.all(np.isfinite(InitialState)):
        raise ValueError("InitialState must be finite")
    DelaySteps = int(np.rint(Delay / Dt))
    State = np.empty((len(Time),) + np.shape(InitialState), dtype=float)
    State[0] = InitialState
    for Index in range(len(Time) - 1):
        DelayedState = InitialState if Index < DelaySteps else State[Index - DelaySteps]
        State[Index + 1] = State[Index] - Dt * Gain * (Laplacian @ DelayedState)
        if not np.all(np.isfinite(State[Index + 1])):
            raise FloatingPointError("Delay integration diverged")
    return Time, State
