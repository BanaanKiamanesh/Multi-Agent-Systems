"""Three-Dimensional Coverage: mathematical model and numerical routines."""

import numpy as np
from scipy.spatial.distance import cdist


class LloydCoverage3D:
    """Density-weighted three-dimensional sampled Voronoi coverage."""

    def __init__(self, Bounds=((0, 8), (0, 8), (0, 6)), GridSize=17, Gain=1.0):
        self.Bounds = np.asarray(Bounds, dtype=float)
        if (
            self.Bounds.shape != (3, 2)
            or not np.isfinite(self.Bounds).all()
            or np.any(self.Bounds[:, 1] <= self.Bounds[:, 0])
        ):
            raise ValueError("Bounds must be three finite increasing intervals")
        if (
            GridSize < 2
            or int(GridSize) != GridSize
            or not np.isfinite(Gain)
            or Gain <= 0
        ):
            raise ValueError(
                "Use at least two grid samples per axis and a positive finite gain"
            )
        Grid = np.meshgrid(
            *[np.linspace(*Interval, int(GridSize)) for Interval in self.Bounds],
            indexing="ij",
        )
        self.Samples = np.column_stack([Axis.ravel() for Axis in Grid])
        self.Density = (
            0.5
            + 1.8 * np.exp(-np.sum((self.Samples - [2, 6, 4]) ** 2, axis=1) / 8)
            + 1.4 * np.exp(-np.sum((self.Samples - [6, 2, 2]) ** 2, axis=1) / 10)
        )
        self.Gain = Gain

    def Centroids(self, Position):
        Owner = np.argmin(cdist(self.Samples, Position), axis=1)
        Mass = np.bincount(Owner, weights=self.Density, minlength=len(Position))
        Weighted = np.column_stack(
            [
                np.bincount(
                    Owner,
                    weights=self.Density * self.Samples[:, Axis],
                    minlength=len(Position),
                )
                for Axis in range(3)
            ]
        )
        return np.where(
            Mass[:, None] > 0, Weighted / np.maximum(Mass[:, None], 1e-12), Position
        )

    def CoverageCost(self, Position):
        MinDist = np.min(cdist(self.Samples, Position), axis=1)
        return float(np.average(MinDist**2, weights=self.Density))

    def SysDyn(self, TimeValue, Position):
        return self.Gain * (self.Centroids(Position) - Position)


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
