"""Switching-Topology Consensus: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


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


class SwitchingTopologyConsensus:
    """First-order consensus under periodic switching of connected graphs.

    Each Laplacian uses the receiver-row convention. For undirected connected
    graphs, switching preserves the common arithmetic mean.
    """

    def __init__(self, Laplacians, Gain=1.5, SwitchPeriod=0.9):
        self.Laplacians = np.asarray(Laplacians, dtype=float)
        if (
            self.Laplacians.ndim != 3
            or self.Laplacians.shape[1] != self.Laplacians.shape[2]
        ):
            raise ValueError(
                "Laplacians must have shape (GraphNum, AgentNum, AgentNum)"
            )
        if Gain <= 0 or SwitchPeriod <= 0:
            raise ValueError("Gain and SwitchPeriod must be positive")
        self.Gain = Gain
        self.SwitchPeriod = SwitchPeriod

    def SysDyn(self, TimeValue, State):
        GraphIndex = int(np.floor(max(TimeValue, 0.0) / self.SwitchPeriod)) % len(
            self.Laplacians
        )
        return -self.Gain * (self.Laplacians[GraphIndex] @ State)


def WeightedLaplacian(Graph):
    """Return receiver-row Laplacian and sorted nodes; L @ X is local disagreement."""
    Nodes = list(sorted(Graph.nodes()))
    Weights = nx.to_numpy_array(Graph, nodelist=Nodes, weight="weight", dtype=float)
    if Graph.is_directed():
        Weights = Weights.T
    if not np.all(np.isfinite(Weights)) or np.any(Weights < 0):
        raise ValueError("Communication weights must be finite and nonnegative")
    return np.diag(np.sum(Weights, axis=1)) - Weights, Nodes
