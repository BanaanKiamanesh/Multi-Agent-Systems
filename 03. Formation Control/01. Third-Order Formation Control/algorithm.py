"""Third-Order Formation Control: mathematical model and numerical routines."""

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


class FormationController:
    """Displacement-based third-order formation, invariant to global translation."""

    def __init__(
        self,
        Laplacian,
        Offsets,
        PositionGain=2.0,
        VelocityGain=4.2,
        AccelerationGain=3.5,
    ):
        self.Laplacian = Laplacian
        self.Offsets = np.asarray(Offsets, dtype=float)
        self.PositionGain = PositionGain
        self.VelocityGain = VelocityGain
        self.AccelerationGain = AccelerationGain
        ValidateIntegratorGains(
            Laplacian, [PositionGain, VelocityGain, AccelerationGain]
        )

    def SysDyn(self, TimeValue, State):
        Error = State[:, 0] - self.Offsets
        Derivative = np.empty_like(State)
        Derivative[:, :2] = State[:, 1:3]
        Derivative[:, 2] = (
            -self.PositionGain * (self.Laplacian @ Error)
            - self.VelocityGain * State[:, 1]
            - self.AccelerationGain * State[:, 2]
        )
        return Derivative


def FormationOffsets(AgentNum=5, Radius=1.4):
    Angle = np.arange(AgentNum) * 2.0 * np.pi / AgentNum
    return Radius * np.column_stack([np.cos(Angle), np.sin(Angle)])


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
