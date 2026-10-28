"""Riccati Linear Synchronization: mathematical model and numerical routines."""

import networkx as nx
import numpy as np
from scipy.linalg import solve_continuous_are


def WeightedLaplacian(Graph):
    """Return receiver-row Laplacian and sorted nodes; L @ X is local disagreement."""
    Nodes = list(sorted(Graph.nodes()))
    Weights = nx.to_numpy_array(Graph, nodelist=Nodes, weight="weight", dtype=float)
    if Graph.is_directed():
        Weights = Weights.T
    if not np.all(np.isfinite(Weights)) or np.any(Weights < 0):
        raise ValueError("Communication weights must be finite and nonnegative")
    return np.diag(np.sum(Weights, axis=1)) - Weights, Nodes


class RiccatiSynchronization:
    """Nominal state-feedback synchronization of identical general linear agents.

    CARE supplies K=B^T P; coupling is scaled above 1/(2*lambda2). This is a
    nominal special case, not an H-infinity observer/uncertainty certificate.
    """

    def __init__(self, Graph, A, B, Coupling=None):
        if Graph.is_directed() or not nx.is_connected(Graph):
            raise ValueError("Use a connected undirected graph")
        self.Laplacian, _ = WeightedLaplacian(Graph)
        self.A, self.B = np.asarray(A, dtype=float), np.asarray(B, dtype=float)
        if (
            self.A.ndim != 2
            or self.B.ndim != 2
            or self.A.shape[0] != self.A.shape[1]
            or self.B.shape[0] != len(self.A)
        ):
            raise ValueError("Incompatible state-space matrices")
        self.P = solve_continuous_are(
            self.A, self.B, np.eye(len(self.A)), np.eye(self.B.shape[1])
        )
        self.K = self.B.T @ self.P
        Lambda = np.linalg.eigvalsh(self.Laplacian)[1]
        if Lambda <= 1e-10:
            raise ValueError("Positive-weight communication must connect all agents")
        self.Coupling = 1.0 / Lambda if Coupling is None else Coupling
        if self.Coupling <= 0.5 / Lambda:
            raise ValueError("Coupling must exceed the CARE synchronization bound")

    def SysDyn(self, TimeValue, State):
        return (
            State @ self.A.T
            - self.Coupling * (self.Laplacian @ State) @ self.K.T @ self.B.T
        )


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
