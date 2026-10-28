"""Bearing Formation Maneuvering: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


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


class BearingFormation:
    """Desired oriented bearings; gradient (range-aware) or bearing-only control."""

    def __init__(self, Graph, DesiredPositions, Gain=2.0, Gradient=False):
        self.DesiredPositions = np.asarray(DesiredPositions, dtype=float)
        self.Edges = FrameworkEdges(Graph, self.DesiredPositions)
        Delta = (
            self.DesiredPositions[self.Edges[:, 1]]
            - self.DesiredPositions[self.Edges[:, 0]]
        )
        Length = np.linalg.norm(Delta, axis=1)
        if np.any(Length <= 1e-10) or Gain <= 0:
            raise ValueError("Desired edges must be nonzero and gain positive")
        self.Bearings = Delta / Length[:, None]
        self.Gain, self.Gradient = Gain, Gradient

    def SysDyn(self, TimeValue, Position):
        Delta = Position[self.Edges[:, 1]] - Position[self.Edges[:, 0]]
        Length = np.linalg.norm(Delta, axis=1)
        if np.any(Length <= 1e-9):
            raise ValueError("Bearing undefined for coincident edge endpoints")
        Current = Delta / Length[:, None]
        Projection = (
            self.Bearings - np.sum(Current * self.Bearings, axis=1)[:, None] * Current
        )
        Force = self.Gain * Projection
        if self.Gradient:
            Force = Force / Length[:, None]
        Rate = np.zeros_like(Position)
        np.add.at(Rate, self.Edges[:, 0], -Force)
        np.add.at(Rate, self.Edges[:, 1], Force)
        return Rate


def BearingLaplacian(Edges, Bearings, AgentNum):
    """Block Laplacian using orthogonal projectors I-g*g^T."""
    Bearings = np.asarray(Bearings, dtype=float)
    if Bearings.shape != (len(Edges), 2) or not np.all(np.isfinite(Bearings)):
        raise ValueError("Provide a finite planar unit bearing per edge")
    if not np.allclose(np.linalg.norm(Bearings, axis=1), 1.0):
        raise ValueError("Bearings must be unit vectors")
    Matrix = np.zeros((2 * AgentNum, 2 * AgentNum))
    for (One, Two), Bearing in zip(Edges, Bearings):
        Projection = np.eye(2) - np.outer(Bearing, Bearing)
        First = slice(2 * One, 2 * One + 2)
        Second = slice(2 * Two, 2 * Two + 2)
        Matrix[First, First] += Projection
        Matrix[Second, Second] += Projection
        Matrix[First, Second] -= Projection
        Matrix[Second, First] -= Projection
    return Matrix


class AnchoredBearingNetwork:
    """Fixed measured/desired bearings for localization or pinned formation.

    Rejects singular follower blocks: two anchors alone do not ensure localizability.
    """

    def __init__(self, Graph, Bearings, AnchorIndices, AnchorPositions, Gain=2.0):
        self.Edges = FrameworkEdges(Graph, np.zeros((len(Graph), 2)))
        self.Matrix = BearingLaplacian(self.Edges, Bearings, len(Graph))
        self.AnchorIndices = np.asarray(AnchorIndices, dtype=int)
        self.AnchorPositions = np.asarray(AnchorPositions, dtype=float)
        if (
            self.AnchorIndices.ndim != 1
            or len(np.unique(self.AnchorIndices)) != len(self.AnchorIndices)
            or np.any(self.AnchorIndices < 0)
            or np.any(self.AnchorIndices >= len(Graph))
            or self.AnchorPositions.shape != (len(self.AnchorIndices), 2)
            or Gain <= 0
        ):
            raise ValueError("Invalid anchors or gain")
        self.FollowerIndices = np.setdiff1d(np.arange(len(Graph)), self.AnchorIndices)
        Dofs = (2 * self.FollowerIndices[:, None] + np.arange(2)).ravel()
        if (
            not len(Dofs)
            or np.linalg.eigvalsh(self.Matrix[np.ix_(Dofs, Dofs)])[0] <= 1e-9
        ):
            raise ValueError(
                "Follower bearing block is singular: network is not localizable"
            )
        self.Gain = Gain

    def SysDyn(self, TimeValue, Position):
        Position = np.array(Position, dtype=float, copy=True)
        Position[self.AnchorIndices] = self.AnchorPositions
        Rate = -self.Gain * (self.Matrix @ Position.ravel()).reshape(Position.shape)
        Rate[self.AnchorIndices] = 0.0
        return Rate


class BearingManeuver:
    """Zhao-Zelazo PI bearing formation maneuvering (2015, Eq. 2)."""

    def __init__(
        self,
        Graph,
        Bearings,
        AnchorIndices,
        AnchorPositions,
        LeaderVelocity,
        ProportionalGain=3.0,
        IntegralGain=1.5,
    ):
        Network = AnchoredBearingNetwork(
            Graph, Bearings, AnchorIndices, AnchorPositions
        )
        if min(ProportionalGain, IntegralGain) <= 0:
            raise ValueError("Gains must be positive")
        self.Matrix = Network.Matrix
        self.AnchorIndices, self.AnchorPositions = (
            Network.AnchorIndices,
            Network.AnchorPositions,
        )
        self.LeaderVelocity = np.asarray(LeaderVelocity, dtype=float)
        if self.LeaderVelocity.shape != self.AnchorPositions.shape:
            raise ValueError("Provide a planar velocity per leader")
        self.ProportionalGain, self.IntegralGain = ProportionalGain, IntegralGain

    def SysDyn(self, TimeValue, State):
        Position = State[:, 0].copy()
        Position[self.AnchorIndices] = (
            self.AnchorPositions + TimeValue * self.LeaderVelocity
        )
        Residual = (self.Matrix @ Position.ravel()).reshape(Position.shape)
        Rate = np.empty_like(State)
        Rate[:, 0] = -self.ProportionalGain * Residual - self.IntegralGain * State[:, 1]
        Rate[:, 1] = Residual
        Rate[self.AnchorIndices, 0] = self.LeaderVelocity
        Rate[self.AnchorIndices, 1] = 0.0
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
