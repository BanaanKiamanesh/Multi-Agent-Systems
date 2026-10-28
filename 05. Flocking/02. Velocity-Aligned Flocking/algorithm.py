"""Velocity-Aligned Flocking: mathematical model and numerical routines."""

import numpy as np


class FlockingController:
    """Olfati-Saber free-space algorithm 2: sigma potential, alignment and gamma agent.

    Smooth finite-range sensing replaces the fixed Morse interaction graph.
    Obstacle beta agents from algorithm 3 are outside this free-space example.
    """

    def __init__(
        self,
        Distance=1.2,
        SensingRange=1.8,
        Epsilon=0.1,
        PositionGain=0.08,
        VelocityGain=1.2,
        ReferenceVelocity=(0.4, 0.1),
    ):
        if (
            min(Distance, Epsilon, PositionGain, VelocityGain) <= 0
            or SensingRange <= Distance
        ):
            raise ValueError(
                "Use positive parameters and sensing range above desired spacing"
            )
        self.Distance, self.SensingRange, self.Epsilon = Distance, SensingRange, Epsilon
        self.PositionGain, self.VelocityGain = PositionGain, VelocityGain
        self.ReferenceVelocity = np.asarray(ReferenceVelocity, dtype=float)

    def SigmaNorm(self, LengthSquared):
        return (np.sqrt(1 + self.Epsilon * LengthSquared) - 1) / self.Epsilon

    def SysDyn(self, TimeValue, State):
        Position, Velocity = State[:, 0], State[:, 1]
        Delta = Position[None, :, :] - Position[:, None, :]
        Squared = np.sum(Delta**2, axis=2)
        Sigma = self.SigmaNorm(Squared)
        Ratio = Sigma / self.SigmaNorm(self.SensingRange**2)
        Bump = np.where(
            Ratio < 0.2,
            1.0,
            np.where(Ratio < 1.0, 0.5 * (1 + np.cos(np.pi * (Ratio - 0.2) / 0.8)), 0.0),
        )
        np.fill_diagonal(Bump, 0.0)
        Argument = Sigma - self.SigmaNorm(self.Distance**2)
        Phi = 5 * Argument / np.sqrt(1 + Argument**2)
        Gradient = Delta / np.sqrt(1 + self.Epsilon * Squared)[:, :, None]
        Acceleration = np.sum((Bump * Phi)[:, :, None] * Gradient, axis=1)
        Acceleration += np.sum(
            Bump[:, :, None] * (Velocity[None, :, :] - Velocity[:, None, :]), axis=1
        )
        Acceleration -= self.PositionGain * (
            Position - TimeValue * self.ReferenceVelocity
        )
        Acceleration -= self.VelocityGain * (Velocity - self.ReferenceVelocity)
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
