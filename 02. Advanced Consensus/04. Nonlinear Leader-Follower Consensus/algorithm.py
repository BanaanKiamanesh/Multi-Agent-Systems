"""Nonlinear Leader-Follower Consensus: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


class NonlinearLeaderFollower:
    """Nonlinear consensus with a sinusoidal leader and pinning injection."""

    def __init__(
        self,
        Graph,
        Gain=1.8,
        PinningGain=2.5,
        ReferenceOffset=1.0,
        ReferenceAmplitude=0.65,
        ReferenceFrequency=0.32,
    ):
        self.ReferenceOffset = ReferenceOffset
        self.ReferenceAmplitude = ReferenceAmplitude
        self.ReferenceFrequency = ReferenceFrequency
        import networkx as nx

        Nodes = list(sorted(Graph.nodes()))
        Weight = nx.to_numpy_array(Graph, nodelist=Nodes, weight="weight")
        self.Weight = Weight.T if Graph.is_directed() else Weight
        self.Gain = Gain
        self.PinningGain = PinningGain

    def Reference(self, TimeValue):
        return self.ReferenceOffset + self.ReferenceAmplitude * np.sin(
            self.ReferenceFrequency * TimeValue
        )

    def ReferenceRate(self, TimeValue):
        return (
            self.ReferenceAmplitude
            * self.ReferenceFrequency
            * np.cos(self.ReferenceFrequency * TimeValue)
        )

    def SysDyn(self, TimeValue, X):
        Difference = X[:, None] - X[None, :]
        Interaction = np.sum(self.Weight * np.tanh(Difference), axis=1)
        Rates = self.ReferenceRate(TimeValue) - self.Gain * Interaction
        Rates -= self.PinningGain * np.tanh(X - self.Reference(TimeValue))
        Rates[0] = self.ReferenceRate(TimeValue)
        return Rates


def RingGraph(AgentNum=5, Directed=False, Weights=None):
    if AgentNum < 2:
        raise ValueError("AgentNum must be at least two")
    Graph = nx.DiGraph() if Directed else nx.Graph()
    Graph.add_nodes_from(range(AgentNum))
    EdgeNum = AgentNum if (Directed or AgentNum > 2) else 1
    for Index in range(EdgeNum):
        Weight = 1.0 if Weights is None else float(Weights[Index])
        Graph.add_edge(Index, (Index + 1) % AgentNum, weight=Weight)
    return Graph


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
