"""Online Gradient Formation Tracking: mathematical model and numerical routines."""

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


def WeightedLaplacian(Graph):
    """Return receiver-row Laplacian and sorted nodes; L @ X is local disagreement."""
    Nodes = list(sorted(Graph.nodes()))
    Weights = nx.to_numpy_array(Graph, nodelist=Nodes, weight="weight", dtype=float)
    if Graph.is_directed():
        Weights = Weights.T
    if not np.all(np.isfinite(Weights)) or np.any(Weights < 0):
        raise ValueError("Communication weights must be finite and nonnegative")
    return np.diag(np.sum(Weights, axis=1)) - Weights, Nodes


class OnlineFormationController:
    """Lecture 8 local gradient/gradient-rate PD control with neighbor observer banks."""

    def __init__(
        self,
        Graph,
        DesiredPositions,
        ObserverGain=8.0,
        ReferenceVelocity=(0.25, -0.1),
        CentroidPositionGain=1.5,
        CentroidVelocityGain=2.5,
        FormationGain=0.5,
        FormationRateGain=0.4,
        AlignmentGain=0.6,
    ):
        self.CentroidPositionGain = CentroidPositionGain
        self.CentroidVelocityGain = CentroidVelocityGain
        self.FormationGain = FormationGain
        self.FormationRateGain = FormationRateGain
        self.AlignmentGain = AlignmentGain
        self.DesiredPositions = np.asarray(DesiredPositions, dtype=float)
        self.Edges = FrameworkEdges(Graph, self.DesiredPositions)
        self.Laplacian, _ = WeightedLaplacian(Graph)
        self.AgentNum = len(Graph)
        self.ObserverGain = ObserverGain
        self.Known = (np.diag(np.diag(self.Laplacian)) - self.Laplacian) > 0
        np.fill_diagonal(self.Known, True)
        self.SquaredDistances = np.sum(
            (
                self.DesiredPositions[self.Edges[:, 0]]
                - self.DesiredPositions[self.Edges[:, 1]]
            )
            ** 2,
            axis=1,
        )
        self.ReferenceVelocity = np.asarray(ReferenceVelocity, dtype=float)

    def InitialState(self, Physical):
        Position = np.broadcast_to(
            Physical[:, 0], (self.AgentNum, self.AgentNum, 2)
        ).copy()
        Velocity = np.broadcast_to(Physical[:, 1], Position.shape).copy()
        Position[~self.Known], Velocity[~self.Known] = 0.0, 0.0
        return np.concatenate([Physical.ravel(), Position.ravel(), Velocity.ravel()])

    def Unpack(self, Flat):
        Boundary = 4 * self.AgentNum
        Size = 2 * self.AgentNum**2
        Physical = Flat[:Boundary].reshape(self.AgentNum, 2, 2)
        PositionBank = Flat[Boundary : Boundary + Size].reshape(
            self.AgentNum, self.AgentNum, 2
        )
        VelocityBank = Flat[Boundary + Size :].reshape(self.AgentNum, self.AgentNum, 2)
        return Physical, PositionBank, VelocityBank

    def Control(self, TimeValue, Physical, PositionBank, VelocityBank):
        Position, Velocity = Physical[:, 0], Physical[:, 1]
        Delta = Position[self.Edges[:, 0]] - Position[self.Edges[:, 1]]
        RelativeVelocity = Velocity[self.Edges[:, 0]] - Velocity[self.Edges[:, 1]]
        Error = np.sum(Delta**2, axis=1) - self.SquaredDistances
        EdgeGradient = Error[:, None] * Delta
        EdgeRate = (
            2 * np.sum(Delta * RelativeVelocity, axis=1)[:, None] * Delta
            + Error[:, None] * RelativeVelocity
        )
        Gradient, GradientRate = np.zeros_like(Position), np.zeros_like(Position)
        for Array, EdgeValues in ((Gradient, EdgeGradient), (GradientRate, EdgeRate)):
            np.add.at(Array, self.Edges[:, 0], EdgeValues)
            np.add.at(Array, self.Edges[:, 1], -EdgeValues)
        CentroidError = PositionBank.mean(axis=1) - TimeValue * self.ReferenceVelocity
        CentroidRate = VelocityBank.mean(axis=1) - self.ReferenceVelocity
        return (
            -self.CentroidPositionGain * CentroidError
            - self.CentroidVelocityGain * CentroidRate
            - self.FormationGain * Gradient
            - self.FormationRateGain * GradientRate
            - self.AlignmentGain * self.Laplacian @ Velocity
        )

    def SysDyn(self, TimeValue, Flat):
        Physical, PositionBank, VelocityBank = self.Unpack(Flat)
        Acceleration = self.Control(TimeValue, Physical, PositionBank, VelocityBank)
        PositionRate = (
            -self.ObserverGain * np.einsum("ij,jkd->ikd", self.Laplacian, PositionBank)
            + VelocityBank
        )
        VelocityRate = -self.ObserverGain * np.einsum(
            "ij,jkd->ikd", self.Laplacian, VelocityBank
        )
        Index = np.arange(self.AgentNum)
        PositionRate[Index, Index] -= self.ObserverGain * (
            PositionBank[Index, Index] - Physical[:, 0]
        )
        VelocityRate[Index, Index] -= self.ObserverGain * (
            VelocityBank[Index, Index] - Physical[:, 1]
        )
        # Actual acceleration is communicated only within the closed neighborhood.
        VelocityRate += self.Known[:, :, None] * Acceleration[None, :, :]
        PhysicalRate = np.stack([Physical[:, 1], Acceleration], axis=1)
        return np.concatenate(
            [PhysicalRate.ravel(), PositionRate.ravel(), VelocityRate.ravel()]
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
