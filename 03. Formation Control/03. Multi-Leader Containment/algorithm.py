"""Multi-Leader Containment: mathematical model and numerical routines."""

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


class MultiLeaderContainment:
    """Stationary leader-follower containment over a directed communication graph.

    The leaders remain fixed. Each follower is governed by local positive
    receiver-row Laplacian coupling to other followers and pinned leaders.
    """

    def __init__(self, Graph, LeaderIndices=(0, 1, 2), Gain=1.7):
        self.Laplacian, self.Nodes = WeightedLaplacian(Graph)
        self.LeaderIndices = np.asarray(LeaderIndices, dtype=int)
        if (
            self.LeaderIndices.ndim != 1
            or len(np.unique(self.LeaderIndices)) != len(self.LeaderIndices)
            or np.any(self.LeaderIndices < 0)
            or np.any(self.LeaderIndices >= len(self.Nodes))
        ):
            raise ValueError("LeaderIndices must be distinct valid array indices")
        self.FollowerIndices = np.setdiff1d(
            np.arange(len(self.Nodes)), self.LeaderIndices
        )
        if not len(self.LeaderIndices) or not len(self.FollowerIndices):
            raise ValueError("Specify at least one leader and one follower")
        if Gain <= 0:
            raise ValueError("Gain must be positive")
        Followers = self.Laplacian[np.ix_(self.FollowerIndices, self.FollowerIndices)]
        if np.min(np.linalg.eigvals(Followers).real) <= 1e-10:
            raise ValueError(
                "Every follower must be reachable from a leader through positive-weight edges"
            )
        self.Gain = Gain

    def SysDyn(self, TimeValue, State):
        Derivative = -self.Gain * (self.Laplacian @ State)
        Derivative[self.LeaderIndices] = 0.0
        return Derivative


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
