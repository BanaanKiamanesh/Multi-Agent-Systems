"""Event-Triggered Consensus: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


def ConnectedGraph(AgentNum=6):
    Graph = nx.cycle_graph(AgentNum)
    Graph.add_edges_from((Index, Index + 2) for Index in range(0, AgentNum - 2, 2))
    nx.set_edge_attributes(Graph, 1.0, "weight")
    return Graph


def TimeGrid(SimTime, Dt):
    """Uniform samples ending exactly at SimTime, with step no larger than Dt."""
    if not np.isfinite(SimTime) or not np.isfinite(Dt) or min(SimTime, Dt) <= 0:
        raise ValueError("SimTime and Dt must be finite and positive")
    return np.linspace(0.0, SimTime, int(np.ceil(SimTime / Dt)) + 1)


def WeightedLaplacian(Graph):
    """Return receiver-row Laplacian and sorted nodes; L @ X is local disagreement."""
    Nodes = list(sorted(Graph.nodes()))
    Weights = nx.to_numpy_array(Graph, nodelist=Nodes, weight="weight", dtype=float)
    if Graph.is_directed():
        Weights = Weights.T
    if not np.all(np.isfinite(Weights)) or np.any(Weights < 0):
        raise ValueError("Communication weights must be finite and nonnegative")
    return np.diag(np.sum(Weights, axis=1)) - Weights, Nodes


def SimulateEventTriggeredConsensus(
    Graph,
    InitialState,
    SimTime=14.0,
    Dt=0.02,
    Gain=1.4,
    Threshold=0.03,
    Decay=0.3,
    MinimumThreshold=0.001,
):
    """Periodic threshold evaluation with asynchronous sample-and-hold broadcasts.

    Each agent broadcasts only if its local deviation from the last broadcast
    exceeds Threshold * exp(-Decay*t) + MinimumThreshold. Controllers use
    current local state and the most recently received neighbor broadcasts.
    The method is periodic event checking, not a continuous event detector.
    """
    Laplacian, Nodes = WeightedLaplacian(Graph)
    Weights = np.diag(np.diag(Laplacian)) - Laplacian
    Degree = np.sum(Weights, axis=1)
    State = np.asarray(InitialState, dtype=float).copy()
    if State.shape != (len(Nodes),):
        raise ValueError("InitialState must be one scalar per agent")
    if Gain <= 0 or min(Threshold, Decay, MinimumThreshold) < 0:
        raise ValueError("Use a positive gain and nonnegative threshold parameters")
    Time = TimeGrid(SimTime, Dt)
    StepNum = len(Time) - 1
    ActualDt = Time[1] - Time[0]
    Motion = np.empty((StepNum + 1, len(State)), dtype=float)
    Motion[0] = State
    Sent = State.copy()
    Events = np.zeros((StepNum + 1, len(State)), dtype=bool)
    Events[0] = True
    for StepIndex in range(StepNum):
        LocalThreshold = Threshold * np.exp(-Decay * Time[StepIndex]) + MinimumThreshold
        Trigger = np.abs(State - Sent) >= LocalThreshold
        Sent[Trigger] = State[Trigger]
        Events[StepIndex] |= Trigger
        Rate = -Gain * (Degree * State - Weights @ Sent)
        State = State + ActualDt * Rate
        Motion[StepIndex + 1] = State
    return Time, Motion, Events
