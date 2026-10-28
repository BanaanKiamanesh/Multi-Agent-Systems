"""Local Network State Observer: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


def FormationOffsets(AgentNum=5, Radius=1.4):
    Angle = np.arange(AgentNum) * 2.0 * np.pi / AgentNum
    return Radius * np.column_stack([np.cos(Angle), np.sin(Angle)])


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


def WeightedLaplacian(Graph):
    """Return receiver-row Laplacian and sorted nodes; L @ X is local disagreement."""
    Nodes = list(sorted(Graph.nodes()))
    Weights = nx.to_numpy_array(Graph, nodelist=Nodes, weight="weight", dtype=float)
    if Graph.is_directed():
        Weights = Weights.T
    if not np.all(np.isfinite(Weights)) or np.any(Weights < 0):
        raise ValueError("Communication weights must be finite and nonnegative")
    return np.diag(np.sum(Weights, axis=1)) - Weights, Nodes


class LocalStateObserver:
    """Lecture 8 pinned consensus bank: one whole-network estimate per observer.

    Only observer i's own signal is pinned. A common known drift may be supplied
    as feedforward; arbitrary unknown remote derivatives are not assumed known.
    """

    def __init__(self, Graph, Signal, Drift=None, Gain=3.0):
        MetropolisWeights(Graph)
        self.Laplacian, _ = WeightedLaplacian(Graph)
        if Gain <= 0:
            raise ValueError("Gain must be positive")
        self.Signal, self.Drift, self.Gain = Signal, Drift, Gain

    def SysDyn(self, TimeValue, Estimate):
        Rate = -self.Gain * np.einsum("ij,jkd->ikd", self.Laplacian, Estimate)
        Index = np.arange(len(self.Laplacian))
        Rate[Index, Index] -= self.Gain * (
            Estimate[Index, Index] - self.Signal(TimeValue)
        )
        if self.Drift is not None:
            Rate += self.Drift(TimeValue)
        return Rate


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
