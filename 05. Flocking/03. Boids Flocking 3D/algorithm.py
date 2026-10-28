"""Boids Flocking 3D: mathematical model and numerical routines."""

import numpy as np


def PairGeometry(Position):
    Delta = Position[None, :, :] - Position[:, None, :]
    return Delta, np.linalg.norm(Delta, axis=-1)


class BoidsController:
    """Archive separation/alignment/cohesion and goal PD, in two or three dimensions.

    Neighborhood averages include self, as Individual.GetNeighbors does. The
    separation term sums unit outward directions, not inverse-square forces.
    """

    def __init__(
        self,
        Reference,
        Radius=2.5,
        SeparationGain=0.16,
        AlignmentGain=0.6,
        CohesionGain=0.25,
        GoalGain=0.5,
    ):
        if (
            Radius <= 0
            or min(SeparationGain, AlignmentGain, CohesionGain, GoalGain) < 0
        ):
            raise ValueError("Use a positive radius and nonnegative gains")
        self.Reference, self.Radius = Reference, Radius
        self.SeparationGain, self.AlignmentGain = SeparationGain, AlignmentGain
        self.CohesionGain, self.GoalGain = CohesionGain, GoalGain

    def SysDyn(self, TimeValue, State):
        Position, Velocity = State[:, 0], State[:, 1]
        Delta, Distance = PairGeometry(Position)
        Neighbors = Distance < self.Radius
        Count = Neighbors.sum(axis=1)
        Direction = np.divide(
            Delta,
            Distance[:, :, None],
            out=np.zeros_like(Delta),
            where=Distance[:, :, None] > 1e-8,
        )
        Separation = -self.SeparationGain * np.sum(
            Neighbors[:, :, None] * Direction, axis=1
        )
        Alignment = self.AlignmentGain * (
            (Neighbors @ Velocity) / Count[:, None] - Velocity
        )
        Cohesion = self.CohesionGain * (
            (Neighbors @ Position) / Count[:, None] - Position
        )
        Goal, GoalVelocity = self.Reference(TimeValue)
        Acceleration = (
            Separation
            + Alignment
            + Cohesion
            + self.GoalGain * (Goal - Position)
            + 3 * self.GoalGain * (GoalVelocity - Velocity)
        )
        return np.stack([Velocity, Acceleration], axis=1)


def FlockDiagnostics(Motion, Radius, Obstacles=None):
    """Time histories of physical neighbor links, separation and obstacle clearance."""
    Position = Motion[:, :, 0]
    Delta = Position[:, :, None] - Position[:, None, :]
    Distance = np.linalg.norm(Delta, axis=-1)
    AgentNum = Position.shape[1]
    Adjacency = Distance < Radius
    Adjacency[:, np.arange(AgentNum), np.arange(AgentNum)] = False
    Masked = Distance.copy()
    Masked[:, np.arange(AgentNum), np.arange(AgentNum)] = np.inf
    Result = dict(
        Adjacency=Adjacency,
        MinimumSeparation=np.min(Masked, axis=(1, 2)),
        MeanDegree=np.mean(Adjacency.sum(axis=2), axis=1),
    )
    if Obstacles is not None and len(Obstacles):
        Clearance = (
            np.linalg.norm(
                Position[:, :, None] - Obstacles[None, None, :, :-1], axis=-1
            )
            - Obstacles[None, None, :, -1]
        )
        Result["ObstacleClearance"] = np.min(Clearance, axis=(1, 2))
    return Result


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
