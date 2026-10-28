"""Adaptive Coverage Control: mathematical model and numerical routines."""

import numpy as np
from scipy.spatial.distance import cdist


class LloydCoverage:
    def __init__(
        self,
        Bounds=((0.0, 10.0), (0.0, 10.0)),
        GridSize=46,
        SensingRadius=np.inf,
        Gain=1.3,
    ):
        self.Bounds = np.asarray(Bounds, dtype=float)
        if (
            self.Bounds.shape != (2, 2)
            or not np.isfinite(self.Bounds).all()
            or np.any(self.Bounds[:, 1] <= self.Bounds[:, 0])
        ):
            raise ValueError("Bounds must be two increasing intervals")
        self.SensingRadius = SensingRadius
        self.Gain = Gain
        if (
            GridSize < 2
            or int(GridSize) != GridSize
            or SensingRadius <= 0
            or np.isnan(SensingRadius)
            or not np.isfinite(Gain)
            or Gain <= 0
        ):
            raise ValueError(
                "Use at least two grid samples per axis and positive radius/gain"
            )
        XGrid, YGrid = np.meshgrid(
            *[np.linspace(*Interval, int(GridSize)) for Interval in self.Bounds],
            indexing="ij",
        )
        self.Samples = np.column_stack([XGrid.ravel(), YGrid.ravel()])
        # Fixed spatial density with two areas of importance.
        self.Density = (
            0.35
            + 2.0 * np.exp(-np.sum((self.Samples - [2.5, 7.5]) ** 2, axis=1) / 5)
            + 1.4 * np.exp(-np.sum((self.Samples - [7.6, 3.0]) ** 2, axis=1) / 8)
        )

    def Partition(self, Position):
        Distance = cdist(self.Samples, Position)
        Owner = np.argmin(Distance, axis=1)
        Valid = np.min(Distance, axis=1) <= self.SensingRadius
        return Owner, Valid

    def DensityAt(self, TimeValue):
        return self.Density

    def Centroids(self, Position, TimeValue=0.0):
        Owner, Valid = self.Partition(Position)
        Density = self.DensityAt(TimeValue)
        N = len(Position)
        Mass = np.bincount(Owner[Valid], weights=Density[Valid], minlength=N)
        Weighted = np.stack(
            [
                np.bincount(
                    Owner[Valid],
                    weights=Density[Valid] * self.Samples[Valid, Dimension],
                    minlength=N,
                )
                for Dimension in range(2)
            ],
            axis=1,
        )
        return np.where(
            Mass[:, None] > 0, Weighted / np.maximum(Mass[:, None], 1e-12), Position
        )

    def CoverageCost(self, Position, TimeValue=0.0):
        """Weighted capped squared distance; uncovered samples cost radius squared."""
        Distance = cdist(self.Samples, Position)
        Minimum = np.min(Distance, axis=1)
        Density = self.DensityAt(TimeValue)
        return float(
            np.average(np.minimum(Minimum, self.SensingRadius) ** 2, weights=Density)
        )

    def SysDyn(self, TimeValue, Position):
        return self.Gain * (self.Centroids(Position, TimeValue) - Position)


class AdaptiveCoverage(LloydCoverage):
    """Time-varying spatial density with an online moving importance hotspot."""

    def __init__(self, **Options):
        super().__init__(**Options)
        self.StaticDensity = self.Density.copy()

    def DensityAt(self, TimeValue):
        MovingCenter = np.array(
            [5.0 + 2.3 * np.cos(0.3 * TimeValue), 5.0 + 2.3 * np.sin(0.3 * TimeValue)]
        )
        Hotspot = 3.0 * np.exp(
            -np.sum((self.Samples - MovingCenter) ** 2, axis=1) / 4.0
        )
        return 0.3 * self.StaticDensity + Hotspot


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
