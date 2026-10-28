"""Olfati-Saber Algorithm 1: mathematical model and numerical routines."""

import numpy as np


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


def Bump(Value, H=0.2):
    Value = np.asarray(Value, dtype=float)
    Result = np.zeros_like(Value)
    Result[(Value >= 0) & (Value < H)] = 1.0
    Mask = (Value >= H) & (Value < 1.0)
    Result[Mask] = 0.5 * (1.0 + np.cos(np.pi * (Value[Mask] - H) / (1.0 - H)))
    return Result


def PairGeometry(Position):
    Delta = Position[None, :, :] - Position[:, None, :]
    return Delta, np.linalg.norm(Delta, axis=-1)


def Sigma1(Value):
    return Value / np.sqrt(1.0 + np.asarray(Value) ** 2)


def SigmaNorm(Vector, Epsilon=0.1):
    return (
        np.sqrt(1.0 + Epsilon * np.sum(np.asarray(Vector) ** 2, axis=-1)) - 1.0
    ) / Epsilon


class OlfatiSaberController:
    """Archive Algorithms 1/2: sigma alpha interactions, optional saturated gamma navigation."""

    def __init__(
        self,
        Reference=None,
        Radius=1.8,
        Distance=1.2,
        Epsilon=0.1,
        H=0.2,
        A=5.0,
        B=5.0,
        GradientGain=2.0,
        AlignmentGain=2.0,
        PositionGain=0.8,
        VelocityGain=2.0,
    ):
        if (
            Radius <= Distance
            or Distance <= 0
            or Epsilon <= 0
            or not 0 < H < 1
            or A > B
            or min(A, B) <= 0
            or min(GradientGain, AlignmentGain, PositionGain, VelocityGain) < 0
        ):
            raise ValueError(
                "Use positive geometry, 0<A<=B, 0<H<1, and nonnegative gains"
            )
        self.Reference, self.Radius, self.Distance = Reference, Radius, Distance
        self.Epsilon, self.H, self.A, self.B = Epsilon, H, A, B
        self.C = abs(A - B) / np.sqrt(4 * A * B)
        self.GradientGain, self.AlignmentGain = GradientGain, AlignmentGain
        self.PositionGain, self.VelocityGain = PositionGain, VelocityGain

    def Alpha(self, Position, Velocity):
        Delta, _ = PairGeometry(Position)
        Length = SigmaNorm(Delta, self.Epsilon)
        Radius = SigmaNorm(np.array([self.Radius]), self.Epsilon)
        Desired = SigmaNorm(np.array([self.Distance]), self.Epsilon)
        Weight = Bump(Length / Radius, self.H)
        np.fill_diagonal(Weight, 0.0)
        Phi = 0.5 * (
            (self.A + self.B) * Sigma1(Length - Desired + self.C) + self.A - self.B
        )
        Direction = (
            Delta / np.sqrt(1 + self.Epsilon * np.sum(Delta**2, axis=2))[:, :, None]
        )
        Gradient = self.GradientGain * np.sum(
            (Weight * Phi)[:, :, None] * Direction, axis=1
        )
        Alignment = self.AlignmentGain * (
            Weight @ Velocity - Weight.sum(axis=1)[:, None] * Velocity
        )
        return Gradient + Alignment

    def SysDyn(self, TimeValue, State):
        Position, Velocity = State[:, 0], State[:, 1]
        Acceleration = self.Alpha(Position, Velocity)
        if self.Reference is not None:
            Goal, GoalVelocity = self.Reference(TimeValue)
            Acceleration -= self.PositionGain * Sigma1(
                Position - Goal
            ) + self.VelocityGain * (Velocity - GoalVelocity)
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


def TriangleLattice(Distance=1.2):
    Height = np.sqrt(3) * Distance / 2
    Position = np.array(
        [
            [0.0, 0.0],
            [Distance, 0.0],
            [2 * Distance, 0.0],
            [Distance / 2, Height],
            [1.5 * Distance, Height],
            [Distance, 2 * Height],
        ]
    )
    return Position - Position.mean(axis=0)
