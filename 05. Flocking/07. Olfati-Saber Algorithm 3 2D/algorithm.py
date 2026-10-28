"""Olfati-Saber Algorithm 3 2D: mathematical model and numerical routines."""

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


class ObstacleFlockingController:
    """Practical Algorithm 3 variant in the archive, not the paper's beta-agent law.

    The shifted Euclidean pair law is repulsive throughout its sensing range.
    TangentMode='velocity' matches Flock.m; 'planar' matches Main.m's fixed 2D
    rotation. Exponential obstacle forces use center distance as supplied.
    """

    def __init__(
        self,
        Goal,
        Obstacles,
        Radius=2.0,
        Distance=1.1,
        ObstacleRadius=4.0,
        H=0.2,
        GradientGain=0.6,
        AlignmentGain=1.0,
        ObstacleGain=3.0,
        GoalGain=1.0,
        Damping=0.8,
        SafetyBuffer=0.3,
        TangentGain=0.02,
        TangentMode="velocity",
    ):
        self.Goal = np.asarray(Goal, dtype=float)
        self.Obstacles = np.asarray(Obstacles, dtype=float)
        if self.Goal.ndim != 1 or len(self.Goal) not in (2, 3):
            raise ValueError("Use a planar or spatial goal")
        if self.Obstacles.size == 0:
            self.Obstacles = np.empty((0, len(self.Goal) + 1))
        if (
            self.Obstacles.ndim != 2
            or self.Obstacles.shape[1] != len(self.Goal) + 1
            or np.any(self.Obstacles[:, -1] <= 0)
            or Radius <= Distance
            or Distance <= 0
            or ObstacleRadius <= 0
            or not 0 < H < 1
            or min(
                GradientGain,
                AlignmentGain,
                ObstacleGain,
                GoalGain,
                Damping,
                SafetyBuffer,
                TangentGain,
            )
            < 0
        ):
            raise ValueError("Invalid flock/obstacle geometry or gains")
        if TangentMode not in ("velocity", "planar") or (
            TangentMode == "planar" and len(self.Goal) != 2
        ):
            raise ValueError("The fixed rotated tangent is planar only")
        self.Radius, self.Distance, self.ObstacleRadius, self.H = (
            Radius,
            Distance,
            ObstacleRadius,
            H,
        )
        self.GradientGain, self.AlignmentGain = GradientGain, AlignmentGain
        self.ObstacleGain, self.GoalGain, self.Damping = ObstacleGain, GoalGain, Damping
        self.SafetyBuffer, self.TangentGain, self.TangentMode = (
            SafetyBuffer,
            TangentGain,
            TangentMode,
        )

    def Acceleration(self, Position, Velocity):
        Delta, Distance = PairGeometry(Position)
        Weight = Bump(Distance / self.Radius, self.H)
        np.fill_diagonal(Weight, 0.0)
        Phi = Weight * (
            Sigma1(Distance - self.Distance) - Sigma1(self.Radius - self.Distance)
        )
        # Finite diagonals are essential: 0*(inf/sqrt(1+inf^2)) is still NaN.
        Direction = np.divide(
            Delta,
            Distance[:, :, None],
            out=np.zeros_like(Delta),
            where=Distance[:, :, None] > 1e-8,
        )
        Alpha = self.GradientGain * np.sum(Phi[:, :, None] * Direction, axis=1)
        Alpha += self.AlignmentGain * (
            Weight @ Velocity - Weight.sum(axis=1)[:, None] * Velocity
        )
        Beta = np.zeros_like(Position)
        for Obstacle in self.Obstacles:
            Offset = Position - Obstacle[:-1]
            Length = np.linalg.norm(Offset, axis=1)
            Normal = np.divide(
                Offset,
                Length[:, None],
                out=np.zeros_like(Offset),
                where=Length[:, None] > 1e-8,
            )
            Active = (Length < self.ObstacleRadius) & (Length > 1e-8)
            Repulsion = (
                np.exp(-0.5 * (Length - Obstacle[-1] - self.SafetyBuffer))[:, None]
                * Normal
            )
            if self.TangentMode == "velocity":
                Tangent = Velocity - np.sum(Velocity * Normal, axis=1)[:, None] * Normal
                Norm = np.linalg.norm(Tangent, axis=1)
                Tangent = np.divide(
                    Tangent,
                    Norm[:, None],
                    out=np.zeros_like(Tangent),
                    where=Norm[:, None] > 1e-8,
                )
            else:
                Tangent = np.column_stack([-Normal[:, 1], Normal[:, 0]])
            Beta += Active[:, None] * (Repulsion + self.TangentGain * Tangent)
        ToGoal = self.Goal - Position
        Length = np.linalg.norm(ToGoal, axis=1)
        Gamma = self.GoalGain * ToGoal / (Length[:, None] + 1e-8)
        return Alpha + self.ObstacleGain * Beta + Gamma - self.Damping * Velocity

    def SysDyn(self, TimeValue, State):
        return np.stack(
            [State[:, 1], self.Acceleration(State[:, 0], State[:, 1])], axis=1
        )


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


def SimulateBoundedFlock(
    Model,
    Initial,
    SimTime=60.0,
    Dt=0.02,
    SpeedLimit=1.5,
    ArenaSize=12.0,
    GoalRadius=1.5,
    Majority=0.8,
    Method="euler",
):
    """Source-style speed cap, arena clipping and majority stopping rule.

    Euler mode follows Flock.m (old velocity position update). Verlet mode
    follows Main.m, including its per-step damping after the velocity update.
    """
    if (
        min(SimTime, Dt, SpeedLimit, ArenaSize, GoalRadius) <= 0
        or not 0 < Majority <= 1
        or Method not in ("euler", "verlet")
    ):
        raise ValueError("Invalid bounded simulation configuration")
    State = np.array(Initial, dtype=float, copy=True)
    if (
        State.ndim != 3
        or State.shape[1:] != (2, len(Model.Goal))
        or not np.isfinite(State).all()
    ):
        raise ValueError("Initial state must be finite (AgentNum,2,Dim)")
    if np.any(State[:, 0] < 0) or np.any(State[:, 0] > ArenaSize):
        raise ValueError("Initial positions must be inside the arena")
    if np.any(np.linalg.norm(State[:, 1], axis=1) > SpeedLimit):
        raise ValueError("Initial speeds exceed the speed limit")
    StepNum = int(np.ceil(SimTime / Dt))
    Step = SimTime / StepNum
    Time, Motion = [0.0], [State.copy()]
    for Index in range(StepNum):
        if np.count_nonzero(
            np.linalg.norm(State[:, 0] - Model.Goal, axis=1) < GoalRadius
        ) >= np.ceil(Majority * len(State)):
            break
        Acceleration = Model.Acceleration(State[:, 0], State[:, 1])
        Position = State[:, 0] + Step * State[:, 1]
        if Method == "verlet":
            Position += 0.5 * Step**2 * Acceleration
        Position = np.clip(Position, 0.0, ArenaSize)
        if Method == "verlet":
            NewAcceleration = Model.Acceleration(Position, State[:, 1])
            Velocity = State[:, 1] + 0.5 * Step * (Acceleration + NewAcceleration)
            # A time-scaled damping fraction keeps step refinement meaningful.
            Velocity *= np.exp(-0.2 * Step)
        else:
            Velocity = State[:, 1] + Step * Acceleration
        Speed = np.linalg.norm(Velocity, axis=1)
        Velocity *= np.minimum(1.0, SpeedLimit / np.maximum(Speed, 1e-12))[:, None]
        State = np.stack([Position, Velocity], axis=1)
        if not np.isfinite(State).all():
            raise FloatingPointError("Nonfinite flock state")
        Time.append((Index + 1) * Step)
        Motion.append(State.copy())
    return np.array(Time), np.array(Motion)
