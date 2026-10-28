"""Moving Leader-Follower Formation: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


def ConnectedGraph(AgentNum=6):
    Graph = nx.cycle_graph(AgentNum)
    Graph.add_edges_from((Index, Index + 2) for Index in range(0, AgentNum - 2, 2))
    nx.set_edge_attributes(Graph, 1.0, "weight")
    return Graph


def FormationOffsets(AgentNum=5, Radius=1.4):
    Angle = np.arange(AgentNum) * 2.0 * np.pi / AgentNum
    return Radius * np.column_stack([np.cos(Angle), np.sin(Angle)])


class LeaderFollowerFormation:
    """Moving reference formation with one leader and damped follower dynamics."""

    def __init__(
        self,
        Graph,
        Offsets,
        Gain=2.4,
        Damping=3.0,
        Pinning=1.5,
        ReferenceSpeed=0.5,
        ReferenceAmplitude=1.0,
        ReferenceFrequency=0.25,
        LeaderPositionGain=6.0,
        LeaderVelocityGain=4.0,
    ):
        self.ReferenceSpeed = ReferenceSpeed
        self.ReferenceAmplitude = ReferenceAmplitude
        self.ReferenceFrequency = ReferenceFrequency
        self.LeaderPositionGain = LeaderPositionGain
        self.LeaderVelocityGain = LeaderVelocityGain
        import networkx as nx

        SelfWeight = nx.to_numpy_array(
            Graph, nodelist=sorted(Graph.nodes()), weight="weight"
        )
        self.Weight = SelfWeight.T if Graph.is_directed() else SelfWeight
        self.Offsets = np.asarray(Offsets)
        self.Gain = Gain
        self.Damping = Damping
        self.Pinning = Pinning

    def Reference(self, TimeValue):
        return np.array(
            [
                self.ReferenceSpeed * TimeValue,
                self.ReferenceAmplitude * np.sin(self.ReferenceFrequency * TimeValue),
            ]
        )

    def ReferenceRate(self, TimeValue):
        return np.array(
            [
                self.ReferenceSpeed,
                self.ReferenceAmplitude
                * self.ReferenceFrequency
                * np.cos(self.ReferenceFrequency * TimeValue),
            ]
        )

    def ReferenceAcceleration(self, TimeValue):
        return np.array(
            [
                0.0,
                -self.ReferenceAmplitude
                * self.ReferenceFrequency**2
                * np.sin(self.ReferenceFrequency * TimeValue),
            ]
        )

    def SysDyn(self, TimeValue, State):
        Position, Velocity = State[:, 0], State[:, 1]
        Error = Position - self.Offsets - self.Reference(TimeValue)
        Relative = Error[:, None] - Error[None, :]
        Coupling = np.sum(self.Weight[:, :, None] * Relative, axis=1)
        Acceleration = (
            self.ReferenceAcceleration(TimeValue)
            - self.Gain * Coupling
            - self.Damping * (Velocity - self.ReferenceRate(TimeValue))
            - self.Pinning * Error
        )
        Acceleration[0] = (
            self.ReferenceAcceleration(TimeValue)
            - self.LeaderPositionGain * Error[0]
            - self.LeaderVelocityGain * (Velocity[0] - self.ReferenceRate(TimeValue))
        )
        return np.stack([Velocity, Acceleration], axis=1)


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
