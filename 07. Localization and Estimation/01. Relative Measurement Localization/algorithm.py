"""Relative Measurement Localization: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


def FormationOffsets(AgentNum=5, Radius=1.4):
    Angle = np.arange(AgentNum) * 2.0 * np.pi / AgentNum
    return Radius * np.column_stack([np.cos(Angle), np.sin(Angle)])


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


class RelativeMeasurementEstimator:
    """Gradient least-squares estimation from oriented, possibly noisy differences.

    Measurements[e] = p[j]-p[i]. One known anchor removes translation ambiguity.
    """

    def __init__(self, Graph, Measurements, AnchorPosition, AnchorIndex=0, Gain=1.5):
        self.Nodes = sorted(Graph.nodes())
        self.Edges = FrameworkEdges(Graph, np.zeros((len(Graph), 2)))
        self.Measurements = np.asarray(Measurements, dtype=float)
        self.AnchorPosition = np.asarray(AnchorPosition, dtype=float)
        if self.Measurements.shape != (len(self.Edges), 2):
            raise ValueError("Provide a planar relative measurement per edge")
        if (
            self.AnchorPosition.shape != (2,)
            or AnchorIndex not in range(len(Graph))
            or Gain <= 0
        ):
            raise ValueError("Provide a valid anchor index, position and positive gain")
        self.AnchorIndex = AnchorIndex
        self.Gain = Gain

    def SysDyn(self, TimeValue, Position):
        # The pinned anchor is a boundary value, independent of its state slot.
        Position = np.array(Position, dtype=float, copy=True)
        Position[self.AnchorIndex] = self.AnchorPosition
        Residual = (
            Position[self.Edges[:, 1]] - Position[self.Edges[:, 0]] - self.Measurements
        )
        Rate = np.zeros_like(Position)
        np.add.at(Rate, self.Edges[:, 0], self.Gain * Residual)
        np.add.at(Rate, self.Edges[:, 1], -self.Gain * Residual)
        Rate[self.AnchorIndex] = 0.0
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
