"""Henneberg Rigid Formation: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


class DistanceBasedFormation:
    """Translation/rotation invariant formation with distance-error potentials.

    Edge (i,j) contributes k * (d_ij - desired_ij) to the agent-gradient.
    A connected graph alone is insufficient to ensure rigidity. Demonstrations
    use an infinitesimally rigid wheel framework and nondegenerate starts.
    """

    def __init__(self, Graph, DesiredPositions, Gain=1.5, DistanceFloor=1e-8):
        if Graph.is_directed() or not nx.is_connected(Graph):
            raise ValueError("An undirected connected formation graph is required")
        self.Nodes = sorted(Graph.nodes())
        Index = {Node: Position for Position, Node in enumerate(self.Nodes)}
        self.Edges = np.asarray(
            [[Index[One], Index[Two]] for One, Two in Graph.edges()], dtype=int
        )
        DesiredPositions = np.asarray(DesiredPositions, dtype=float)
        if DesiredPositions.shape != (len(self.Nodes), 2):
            raise ValueError("DesiredPositions must be (AgentNum, 2)")
        self.Distances = np.linalg.norm(
            DesiredPositions[self.Edges[:, 0]] - DesiredPositions[self.Edges[:, 1]],
            axis=1,
        )
        self.Gain = Gain
        self.DistanceFloor = DistanceFloor

    def EdgeError(self, Position):
        Delta = Position[self.Edges[:, 0]] - Position[self.Edges[:, 1]]
        return np.linalg.norm(Delta, axis=-1) - self.Distances

    def SysDyn(self, TimeValue, Position):
        Delta = Position[self.Edges[:, 0]] - Position[self.Edges[:, 1]]
        Length = np.linalg.norm(Delta, axis=-1)
        Force = (
            self.Gain
            * ((Length - self.Distances) / np.maximum(Length, self.DistanceFloor))[
                :, None
            ]
            * Delta
        )
        Rate = np.zeros_like(Position)
        np.add.at(Rate, self.Edges[:, 0], -Force)
        np.add.at(Rate, self.Edges[:, 1], Force)
        return Rate


def HennebergGraph(Operations):
    """Planar minimally rigid graph construction by Henneberg I and II moves.

    Start at edge (0,1). ('I',a,b) adds a node adjacent to a,b;
    ('II',a,b,c) replaces edge (a,b) by a new node connected to a,b,c.
    """
    Graph = nx.Graph([(0, 1)])
    for Operation in Operations:
        if Operation[0] == "I" and len(Operation) == 3:
            Neighbors = Operation[1:]
        elif Operation[0] == "II" and len(Operation) == 4:
            Neighbors = Operation[1:]
            if not Graph.has_edge(Neighbors[0], Neighbors[1]):
                raise ValueError("Henneberg II must split an existing edge")
        else:
            raise ValueError("Unknown Henneberg operation")
        if len(set(Neighbors)) != len(Neighbors) or any(
            Node not in Graph for Node in Neighbors
        ):
            raise ValueError("Select distinct existing neighbor nodes")
        if Operation[0] == "II":
            Graph.remove_edge(Neighbors[0], Neighbors[1])
        NewNode = len(Graph)
        Graph.add_edges_from((NewNode, Node) for Node in Neighbors)
    return Graph


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
