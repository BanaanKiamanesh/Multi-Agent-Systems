"""Connectivity-Preserving Formation: mathematical model and numerical routines."""

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


class ConnectivityPreservingFormation:
    """Fixed-edge distance barriers with a soft displacement target.

    V = 0.5*k*||p-p_des||^2 + beta*sum r^2/(R^2-r^2).
    Finite-step integration must still be checked for communication-range violations.
    """

    def __init__(
        self,
        Graph,
        DesiredPositions,
        CommunicationRange=3.0,
        Gain=0.6,
        BarrierGain=0.15,
    ):
        self.DesiredPositions = np.asarray(DesiredPositions, dtype=float)
        self.Edges = FrameworkEdges(Graph, self.DesiredPositions)
        if CommunicationRange <= 0 or Gain <= 0 or BarrierGain <= 0:
            raise ValueError("Range and gains must be positive")
        self.CommunicationRange = CommunicationRange
        self.Gain, self.BarrierGain = Gain, BarrierGain

    def SysDyn(self, TimeValue, Position):
        Delta = Position[self.Edges[:, 0]] - Position[self.Edges[:, 1]]
        Gap = self.CommunicationRange**2 - np.sum(Delta**2, axis=1)
        if np.any(Gap <= 0):
            raise ValueError("A communication edge reached or exceeded the range")
        Force = (2 * self.BarrierGain * self.CommunicationRange**2 / Gap**2)[
            :, None
        ] * Delta
        Rate = -self.Gain * (Position - self.DesiredPositions)
        np.add.at(Rate, self.Edges[:, 0], -Force)
        np.add.at(Rate, self.Edges[:, 1], Force)
        return Rate


def FormationOffsets(AgentNum=5, Radius=1.4):
    Angle = np.arange(AgentNum) * 2.0 * np.pi / AgentNum
    return Radius * np.column_stack([np.cos(Angle), np.sin(Angle)])


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
