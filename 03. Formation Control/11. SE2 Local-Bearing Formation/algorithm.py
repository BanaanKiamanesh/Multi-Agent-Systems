"""SE2 Local-Bearing Formation: mathematical model and numerical routines."""

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


class SE2BearingFormation:
    """Local-frame bearing-potential gradient for planar positions and headings.

    Both directions of every undirected edge are observed. Translation, scale
    and a simultaneous global position/heading rotation remain unobservable.
    """

    def __init__(
        self,
        Graph,
        DesiredPositions,
        DesiredHeadings,
        PositionGain=2.0,
        HeadingGain=2.0,
    ):
        DesiredPositions = np.asarray(DesiredPositions, dtype=float)
        self.Edges = FrameworkEdges(Graph, DesiredPositions)
        self.DesiredHeadings = np.asarray(DesiredHeadings, dtype=float)
        if (
            self.DesiredHeadings.shape != (len(Graph),)
            or min(PositionGain, HeadingGain) <= 0
        ):
            raise ValueError("Provide a heading per node and positive gains")
        Delta = DesiredPositions[self.Edges[:, 1]] - DesiredPositions[self.Edges[:, 0]]
        Length = np.linalg.norm(Delta, axis=1)
        if np.any(Length <= 1e-9):
            raise ValueError("Desired edge endpoints must be distinct")
        Bearing = Delta / Length[:, None]
        self.LocalTargets = np.stack(
            [
                self.Rotate(Bearing, -self.DesiredHeadings[self.Edges[:, 0]]),
                self.Rotate(-Bearing, -self.DesiredHeadings[self.Edges[:, 1]]),
            ],
            axis=1,
        )
        self.PositionGain, self.HeadingGain = PositionGain, HeadingGain

    @staticmethod
    def Rotate(Vector, Angle):
        Cosine, Sine = np.cos(Angle), np.sin(Angle)
        return np.column_stack(
            [
                Cosine * Vector[:, 0] - Sine * Vector[:, 1],
                Sine * Vector[:, 0] + Cosine * Vector[:, 1],
            ]
        )

    def Geometry(self, State):
        One, Two = self.Edges[:, 0], self.Edges[:, 1]
        Delta = State[Two, :2] - State[One, :2]
        Length = np.linalg.norm(Delta, axis=1)
        if np.any(Length <= 1e-9):
            raise ValueError("Bearing undefined at coincident endpoints")
        Bearing = Delta / Length[:, None]
        Local = np.stack(
            [
                self.Rotate(Bearing, -State[One, 2]),
                self.Rotate(-Bearing, -State[Two, 2]),
            ],
            axis=1,
        )
        return Bearing, Length, Local

    def Potential(self, State):
        return 0.5 * np.sum((self.Geometry(State)[2] - self.LocalTargets) ** 2)

    def SysDyn(self, TimeValue, State):
        Bearing, Length, Local = self.Geometry(State)
        One, Two = self.Edges[:, 0], self.Edges[:, 1]
        WorldDerivative = -self.Rotate(
            self.LocalTargets[:, 0], State[One, 2]
        ) + self.Rotate(self.LocalTargets[:, 1], State[Two, 2])
        Projected = (
            WorldDerivative
            - np.sum(Bearing * WorldDerivative, axis=1)[:, None] * Bearing
        ) / Length[:, None]
        Rate = np.zeros_like(State)
        np.add.at(Rate[:, :2], One, self.PositionGain * Projected)
        np.add.at(Rate[:, :2], Two, -self.PositionGain * Projected)
        Error = Local - self.LocalTargets
        Perpendicular = np.stack([-Local[:, :, 1], Local[:, :, 0]], axis=2)
        HeadingRate = self.HeadingGain * np.sum(Error * Perpendicular, axis=2)
        np.add.at(Rate[:, 2], One, HeadingRate[:, 0])
        np.add.at(Rate[:, 2], Two, HeadingRate[:, 1])
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
