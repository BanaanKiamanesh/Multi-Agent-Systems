"""Fourth-Order Consensus: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


def ConnectedGraph(AgentNum=6):
    Graph = nx.cycle_graph(AgentNum)
    Graph.add_edges_from((Index, Index + 2) for Index in range(0, AgentNum - 2, 2))
    nx.set_edge_attributes(Graph, 1.0, "weight")
    return Graph


def ValidateIntegratorGains(Laplacian, Gains):
    """Require stable modal polynomials, apart from the consensus position mode.

    For a Laplacian eigenvalue lambda the characteristic polynomial is
    s^m + g[m-1]*s^(m-1) + ... + g[1]*s + g[0]*lambda.
    Positive coefficients alone do not imply stability for m >= 3.
    """
    Laplacian = np.asarray(Laplacian, dtype=float)
    Gains = np.asarray(Gains, dtype=float)
    if (
        Laplacian.ndim != 2
        or Laplacian.shape[0] != Laplacian.shape[1]
        or not np.isfinite(Laplacian).all()
        or not np.isfinite(Gains).all()
        or np.any(Gains <= 0)
    ):
        raise ValueError("Use a finite square Laplacian and finite positive gains")
    for Value in np.linalg.eigvals(Laplacian):
        Polynomial = np.r_[1.0, Gains[:0:-1], Gains[0] * Value]
        if abs(Value) < 1e-10:
            Polynomial = Polynomial[:-1]
        if np.max(np.roots(Polynomial).real) >= -1e-10:
            raise ValueError(
                "Gains do not stabilize the Laplacian's integrator-chain modes"
            )


class HighOrderConsensus:
    """Integrator chains with Laplacian position feedback and absolute derivative damping.

    Positions remain translation invariant; higher states converge to zero.
    Shape: (AgentNum, Order, Dimension). All derivatives above order zero
    must be initialized as desired, typically with zero collective drift.
    """

    def __init__(self, Laplacian, Order=2, Gains=None):
        if Order not in (2, 3, 4):
            raise ValueError("Order must be 2, 3, or 4")
        self.Laplacian = Laplacian
        self.Order = Order
        Default = {2: [2.0, 2.5], 3: [1.6, 3.5, 3.4], 4: [1.0, 4.0, 6.0, 4.0]}
        self.Gains = np.array(Default[Order] if Gains is None else Gains, dtype=float)
        if len(self.Gains) != Order:
            raise ValueError("Specify one position gain and Order-1 damping gains")
        ValidateIntegratorGains(Laplacian, self.Gains)

    def SysDyn(self, TimeValue, State):
        Rate = np.empty_like(State)
        Rate[:, :-1] = State[:, 1:]
        Rate[:, -1] = -self.Gains[0] * (self.Laplacian @ State[:, 0])
        Rate[:, -1] -= np.einsum("o,nod->nd", self.Gains[1:], State[:, 1:])
        return Rate


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
