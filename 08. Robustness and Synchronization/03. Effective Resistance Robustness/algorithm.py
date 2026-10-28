"""Effective Resistance Robustness: mathematical model and numerical routines."""

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


def EffectiveResistance(Graph, One, Two):
    """b^T L^+ b for a connected, positive-weight undirected network."""
    if Graph.is_directed() or not nx.is_connected(Graph) or One == Two:
        raise ValueError("Use distinct nodes in a connected undirected graph")
    Laplacian, Nodes = WeightedLaplacian(Graph)
    if any(Data.get("weight", 1.0) <= 0 for _, _, Data in Graph.edges(data=True)):
        raise ValueError("Effective resistance requires positive base weights")
    Difference = np.zeros(len(Nodes))
    Difference[Nodes.index(One)], Difference[Nodes.index(Two)] = 1.0, -1.0
    return float(Difference @ np.linalg.pinv(Laplacian) @ Difference)


class FirstOrderConsensus:
    def __init__(self, Laplacian, Gain=1.25):
        self.Laplacian = Laplacian
        self.Gain = Gain

    def SysDyn(self, TimeValue, X):
        return -self.Gain * (self.Laplacian @ X)


def PerturbedLaplacian(Graph, One, Two, Perturbation):
    """Add delta*b*b^T; a negative delta represents an antagonistic edge."""
    Laplacian, Nodes = WeightedLaplacian(Graph)
    Difference = np.zeros(len(Nodes))
    Difference[Nodes.index(One)], Difference[Nodes.index(Two)] = 1.0, -1.0
    return Laplacian + Perturbation * np.outer(Difference, Difference)


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
