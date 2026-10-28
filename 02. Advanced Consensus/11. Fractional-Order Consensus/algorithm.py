"""Fractional-Order Consensus: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


def ConnectedGraph(AgentNum=6):
    Graph = nx.cycle_graph(AgentNum)
    Graph.add_edges_from((Index, Index + 2) for Index in range(0, AgentNum - 2, 2))
    nx.set_edge_attributes(Graph, 1.0, "weight")
    return Graph


class NonlinearFractionalConsensus:
    """Saturating nonlinear distributed protocol used with a Caputo derivative."""

    def __init__(self, Laplacian, Gain=1.1):
        self.Laplacian = Laplacian
        self.Gain = Gain

    def SysDyn(self, TimeValue, X):
        return -self.Gain * np.tanh(self.Laplacian @ X)


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


def SimulateFractional(Dynamics, InitialState, Order=0.85, SimTime=14, Dt=0.04):
    """Explicit Grunwald-Letnikov approximation of Caputo D^Order(X) = F(X).

    Uses the initialized GL derivative of X-X0; for 0 < Order <= 1 and
    sufficiently smooth solutions it approximates the Caputo derivative.
    Computational cost is quadratic in the time-grid size.
    """
    if (
        not np.all(np.isfinite(Order))
        or np.any(np.asarray(Order) <= 0)
        or np.any(np.asarray(Order) > 1)
    ):
        raise ValueError("Fractional order must be in (0, 1]")
    Time = TimeGrid(SimTime, Dt)
    Dt = Time[1] - Time[0]
    InitialState = np.asarray(InitialState, dtype=float)
    if not np.all(np.isfinite(InitialState)):
        raise ValueError("InitialState must be finite")
    State = np.empty((len(Time),) + np.shape(InitialState), dtype=float)
    State[0] = InitialState
    Orders = np.broadcast_to(np.asarray(Order, dtype=float), np.shape(InitialState))
    Coefficients = np.empty((len(Time),) + np.shape(InitialState), dtype=float)
    Coefficients[0] = 1.0
    for Index in range(1, len(Time)):
        Coefficients[Index] = Coefficients[Index - 1] * (1.0 - (Orders + 1.0) / Index)
    for Index in range(1, len(Time)):
        Memory = np.sum(
            Coefficients[1 : Index + 1] * (State[Index - 1 :: -1] - InitialState),
            axis=0,
        )
        State[Index] = (
            InitialState
            + Dt**Orders * Dynamics(Time[Index - 1], State[Index - 1])
            - Memory
        )
        if not np.all(np.isfinite(State[Index])):
            raise FloatingPointError("Fractional integration diverged")
    return Time, State
