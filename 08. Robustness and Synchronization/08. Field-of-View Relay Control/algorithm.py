"""Field-of-View Relay Control: mathematical model and numerical routines."""

import numpy as np


class FieldOfViewRelay:
    """Fabris-Zelazo two-agent relay switching law (Eq. 9), fixed planar camera axis."""

    def __init__(self, Targets, TargetRates, HalfAngle=0.65, Gain=3.0):
        if not 0 < HalfAngle < np.pi / 2 or Gain <= 0:
            raise ValueError("Use an acute half-angle and positive gain")
        self.Targets, self.TargetRates = Targets, TargetRates
        self.HalfAngle, self.Gain = HalfAngle, Gain
        self.Axis = np.array([1.0, 0.0])

    def RelayRate(self, Relay, Targets):
        Delta = Targets - Relay
        Length = np.linalg.norm(Delta, axis=1)
        if len(Targets) != 2 or np.any(Length <= 1e-8):
            raise ValueError("Two noncoincident targets are required")
        Bearing = Delta / Length[:, None]
        Angle = np.arctan2(Bearing[:, 1], Bearing[:, 0])
        if np.any(np.abs(Angle) > self.HalfAngle + 1e-6):
            raise ValueError("A target left the field of view")
        Projection = self.Axis - (Bearing @ self.Axis)[:, None] * Bearing
        if Bearing[0, 1] * Bearing[1, 1] >= 0:
            return -self.Gain * Projection[np.argmax(np.abs(Angle))]
        return -self.Gain * Projection.sum(axis=0)

    def SysDyn(self, TimeValue, Position):
        Targets = self.Targets(TimeValue)
        return np.vstack(
            [self.RelayRate(Position[0], Targets), self.TargetRates(TimeValue)]
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
