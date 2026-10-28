"""Linear State-Space Consensus: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


def ConnectedGraph(AgentNum=6):
    Graph = nx.cycle_graph(AgentNum)
    Graph.add_edges_from((Index, Index + 2) for Index in range(0, AgentNum - 2, 2))
    nx.set_edge_attributes(Graph, 1.0, "weight")
    return Graph


class LinearStateConsensus:
    """Decentralized consensus of identical 2-state double-integrator agents."""

    def __init__(
        self, Laplacian, PositionGain=1.8, VelocityGain=2.8, NeighborGain=0.35
    ):
        self.Laplacian = Laplacian
        self.PositionGain = PositionGain
        self.VelocityGain = VelocityGain
        self.NeighborGain = NeighborGain
        self.A = np.array([[0.0, 1.0], [0.0, 0.0]])
        self.B = np.array([[0.0], [1.0]])

    def ControlInput(self, X):
        Error = self.Laplacian @ X
        return (
            -self.PositionGain * Error[:, 0]
            - self.NeighborGain * Error[:, 1]
            - self.VelocityGain * X[:, 1]
        )

    def SysDyn(self, TimeValue, X):
        return X @ self.A.T + self.ControlInput(X)[:, None] @ self.B.T


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


def RK4Step(Dynamics, State, TimeValue, Dt):
    K1 = np.asarray(Dynamics(TimeValue, State), dtype=float)
    K2 = np.asarray(Dynamics(TimeValue + Dt / 2, State + Dt * K1 / 2), dtype=float)
    K3 = np.asarray(Dynamics(TimeValue + Dt / 2, State + Dt * K2 / 2), dtype=float)
    K4 = np.asarray(Dynamics(TimeValue + Dt, State + Dt * K3), dtype=float)
    return State + Dt * (K1 + 2 * K2 + 2 * K3 + K4) / 6


def TimeGrid(SimTime, Dt):
    """Uniform samples ending exactly at SimTime, with step no larger than Dt."""
    if not np.isfinite(SimTime) or not np.isfinite(Dt) or min(SimTime, Dt) <= 0:
        raise ValueError("SimTime and Dt must be finite and positive")
    return np.linspace(0.0, SimTime, int(np.ceil(SimTime / Dt)) + 1)


def Simulate(Dynamics, InitialState, SimTime=12.0, Dt=0.02):
    Time = TimeGrid(SimTime, Dt)
    InitialState = np.asarray(InitialState, dtype=float)
    if not np.all(np.isfinite(InitialState)):
        raise ValueError("InitialState must be finite")
    StepDt = Time[1] - Time[0]
    State = np.empty((len(Time),) + np.shape(InitialState), dtype=float)
    State[0] = InitialState
    for Index in range(len(Time) - 1):
        State[Index + 1] = RK4Step(Dynamics, State[Index], Time[Index], StepDt)
        if not np.all(np.isfinite(State[Index + 1])):
            raise FloatingPointError("Nonfinite state detected during integration")
    return Time, State
