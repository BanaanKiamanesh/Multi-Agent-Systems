"""Finite-Time Consensus: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


def ConnectedGraph(AgentNum=6):
    Graph = nx.cycle_graph(AgentNum)
    Graph.add_edges_from((Index, Index + 2) for Index in range(0, AgentNum - 2, 2))
    nx.set_edge_attributes(Graph, 1.0, "weight")
    return Graph


class FiniteTimeConsensus:
    """Continuous non-Lipschitz consensus using sign-power neighbor coupling.

    x_dot_i = -k sum_j a_ij sign(x_i-x_j) abs(x_i-x_j)**alpha.
    For connected undirected graphs and 0 < alpha < 1 this protocol has
    finite-time convergence in continuous time. Numeric integration has
    finite-step residuals close to the non-Lipschitz equilibrium.
    """

    def __init__(self, Graph, Gain=1.8, Alpha=0.6):
        if not 0 < Alpha < 1 or not np.isfinite(Gain) or Gain <= 0:
            raise ValueError("Use 0 < Alpha < 1 and a finite positive gain")
        if (
            not isinstance(Graph, nx.Graph)
            or Graph.is_directed()
            or not nx.is_connected(Graph)
        ):
            raise ValueError("Finite-time demo requires a connected undirected graph")
        Nodes = sorted(Graph.nodes())
        self.Weight = nx.to_numpy_array(Graph, nodelist=Nodes, weight="weight")
        if np.any(self.Weight < 0):
            raise ValueError("Negative weights are unsupported")
        self.Gain = Gain
        self.Alpha = Alpha

    def SysDyn(self, TimeValue, State):
        Difference = State[:, None] - State[None, :]
        Interaction = np.sign(Difference) * np.abs(Difference) ** self.Alpha
        return -self.Gain * np.sum(self.Weight * Interaction, axis=1)


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


def WeightedLaplacian(Graph):
    """Return receiver-row Laplacian and sorted nodes; L @ X is local disagreement."""
    Nodes = list(sorted(Graph.nodes()))
    Weights = nx.to_numpy_array(Graph, nodelist=Nodes, weight="weight", dtype=float)
    if Graph.is_directed():
        Weights = Weights.T
    if not np.all(np.isfinite(Weights)) or np.any(Weights < 0):
        raise ValueError("Communication weights must be finite and nonnegative")
    return np.diag(np.sum(Weights, axis=1)) - Weights, Nodes
