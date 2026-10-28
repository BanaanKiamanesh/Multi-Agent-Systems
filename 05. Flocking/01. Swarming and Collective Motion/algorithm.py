"""Swarming and Collective Motion: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


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


class SwarmController:
    """Morse-type pair attraction/repulsion with viscous velocity damping."""

    def __init__(self, Graph, Repulsion=3.5, Attraction=0.85, Damping=1.8):
        import networkx as nx

        self.Weight = nx.to_numpy_array(
            Graph, nodelist=sorted(Graph.nodes()), weight="weight"
        )
        self.Repulsion = Repulsion
        self.Attraction = Attraction
        self.Damping = Damping

    def SysDyn(self, TimeValue, State):
        Position = State[:, 0]
        Velocity = State[:, 1]
        Difference = Position[:, None, :] - Position[None, :, :]
        Distance = np.linalg.norm(Difference, axis=2)
        np.fill_diagonal(Distance, np.inf)
        ForceSize = self.Repulsion * np.exp(
            -Distance / 0.65
        ) - self.Attraction * np.exp(-Distance / 2.7)
        Force = np.sum(
            self.Weight[:, :, None]
            * ForceSize[:, :, None]
            * Difference
            / np.maximum(Distance[:, :, None], 0.08),
            axis=1,
        )
        Derivative = np.empty_like(State)
        Derivative[:, 0] = Velocity
        Derivative[:, 1] = Force - self.Damping * Velocity
        return Derivative


def nxCompleteGraph(AgentNum):
    Graph = nx.complete_graph(AgentNum)
    nx.set_edge_attributes(Graph, 1.0, "weight")
    return Graph
