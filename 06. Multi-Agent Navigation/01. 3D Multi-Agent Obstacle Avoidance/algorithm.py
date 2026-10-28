"""3D Multi-Agent Obstacle Avoidance: mathematical model and numerical routines."""

import numpy as np


class ObstacleAvoidance3D:
    """Second-order multi-agent navigation with spherical obstacle repulsion.

    Local potential-based control, not a guaranteed collision-free planner.
    """

    def __init__(
        self,
        Goals,
        ObstacleCenters,
        ObstacleRadii,
        GoalGain=1.1,
        Damping=2.0,
        RepulsionGain=2.0,
        AgentRadius=0.16,
        SafeDistance=1.15,
    ):
        self.Goals = np.asarray(Goals, dtype=float)
        self.ObstacleCenters = np.asarray(ObstacleCenters, dtype=float)
        self.ObstacleRadii = np.asarray(ObstacleRadii, dtype=float)
        self.GoalGain = GoalGain
        self.Damping = Damping
        self.RepulsionGain = RepulsionGain
        self.AgentRadius = AgentRadius
        self.SafeDistance = SafeDistance

    def ControlInput(self, Position, Velocity):
        Difference = Position[:, None, :] - self.ObstacleCenters[None, :, :]
        Dist = np.linalg.norm(Difference, axis=2)
        Clearance = Dist - self.ObstacleRadii[None, :] - self.AgentRadius
        ObstacleIntensity = self.RepulsionGain * np.maximum(
            1.0 / np.maximum(Clearance, 0.17) - 1.0 / self.SafeDistance, 0.0
        )
        ObstacleIntensity /= np.maximum(Clearance, 0.17) ** 2
        ObstacleForce = np.sum(
            ObstacleIntensity[:, :, None]
            * Difference
            / np.maximum(Dist[:, :, None], 0.001),
            axis=1,
        )
        DifferenceAgents = Position[:, None] - Position[None, :]
        DistAgents = np.linalg.norm(DifferenceAgents, axis=2)
        np.fill_diagonal(DistAgents, np.inf)
        PairClearance = DistAgents - 2.0 * self.AgentRadius
        PairIntensity = (
            np.maximum(1.0 / np.maximum(PairClearance, 0.16) - 1.0 / 0.85, 0.0)
            / np.maximum(PairClearance, 0.16) ** 2
        )
        PairForce = np.sum(
            0.55
            * PairIntensity[:, :, None]
            * DifferenceAgents
            / np.maximum(DistAgents[:, :, None], 1e-9),
            axis=1,
        )
        Accel = (
            self.GoalGain * (self.Goals - Position)
            - self.Damping * Velocity
            + ObstacleForce
            + PairForce
        )
        return np.clip(Accel, -8.0, 8.0)

    def SysDyn(self, TimeValue, State):
        Output = np.empty_like(State)
        Output[:, 0] = State[:, 1]
        Output[:, 1] = self.ControlInput(State[:, 0], State[:, 1])
        return Output


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
