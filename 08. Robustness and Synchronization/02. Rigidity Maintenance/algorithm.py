"""Rigidity Maintenance: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


def RigidityMatrix(Edges, Position):
    """Jacobian of half the squared edge distances in planar node-major order."""
    Matrix = np.zeros((len(Edges), Position.size))
    for Index, (One, Two) in enumerate(Edges):
        Delta = Position[One] - Position[Two]
        Matrix[Index, 2 * One : 2 * One + 2] = Delta
        Matrix[Index, 2 * Two : 2 * Two + 2] = -Delta
    return Matrix


def RigidityEigenpair(Edges, Position):
    Matrix = RigidityMatrix(Edges, Position)
    Values, Vectors = np.linalg.eigh(Matrix.T @ Matrix)
    # Three rigid-body modes in the plane: two translations and one rotation.
    return float(Values[3]), Vectors[:, 3].reshape(Position.shape)


def FrameworkEdges(Graph, Position):
    """Use sorted-node array indexing consistently with the existing graph tools."""
    Position = np.asarray(Position, dtype=float)
    if len(Graph) < 2 or Graph.is_directed() or not nx.is_connected(Graph):
        raise ValueError("A connected undirected framework is required")
    if Position.shape != (len(Graph), 2) or not np.all(np.isfinite(Position)):
        raise ValueError("Position must be a finite (AgentNum, 2) array")
    Index = {Node: Number for Number, Node in enumerate(sorted(Graph.nodes()))}
    Edges = np.array(
        [[Index[One], Index[Two]] for One, Two in Graph.edges()], dtype=int
    )
    if np.any(Edges[:, 0] == Edges[:, 1]):
        raise ValueError("Framework self-loops are unsupported")
    return Edges


class RigidityMaintenance:
    """Spectral barrier with exact centralized eigenpairs on a fixed planar graph.

    This illustrates the lecture potential, without the distributed eigenvector
    estimator, changing sensing weights or range-only common-frame estimator.
    """

    def __init__(
        self, Graph, DesiredPositions, Gain=0.5, BarrierGain=0.06, MinimumEigenvalue=0.1
    ):
        self.DesiredPositions = np.asarray(DesiredPositions, dtype=float)
        self.Edges = FrameworkEdges(Graph, self.DesiredPositions)
        if len(Graph) < 3 or min(Gain, BarrierGain, MinimumEigenvalue) <= 0:
            raise ValueError("Use at least three agents and positive parameters")
        self.Gain, self.BarrierGain = Gain, BarrierGain
        self.MinimumEigenvalue = MinimumEigenvalue

    def EigenvalueGradient(self, Position):
        Value, Vector = RigidityEigenpair(self.Edges, Position)
        Delta = Position[self.Edges[:, 0]] - Position[self.Edges[:, 1]]
        VectorDelta = Vector[self.Edges[:, 0]] - Vector[self.Edges[:, 1]]
        EdgeGradient = 2 * np.sum(Delta * VectorDelta, axis=1)[:, None] * VectorDelta
        Gradient = np.zeros_like(Position)
        np.add.at(Gradient, self.Edges[:, 0], EdgeGradient)
        np.add.at(Gradient, self.Edges[:, 1], -EdgeGradient)
        return Value, Gradient

    def SysDyn(self, TimeValue, Position):
        Value, Gradient = self.EigenvalueGradient(Position)
        Gap = Value - self.MinimumEigenvalue
        if Gap <= 0:
            raise ValueError("Framework crossed the rigidity barrier")
        return (
            -self.Gain * (Position - self.DesiredPositions)
            + self.BarrierGain * Gradient / Gap**2
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
