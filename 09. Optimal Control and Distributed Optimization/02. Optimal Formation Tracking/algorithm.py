"""Optimal Formation Tracking: mathematical model and numerical routines."""

import networkx as nx
import numpy as np
from scipy.optimize import minimize


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


class OptimalFormationPlanner:
    """Centralized finite-horizon direct shooting for double integrators.

    Minimizes centroid tracking, squared-distance formation error, velocity
    disagreement and input energy. This is a SciPy alternative to PRONTO;
    the nonconvex objective admits local solutions only.
    """

    def __init__(
        self,
        Graph,
        DesiredPositions,
        InitialState,
        SimTime=8.0,
        Dt=0.1,
        ReferenceVelocity=(0.35, -0.15),
        InputWeight=0.15,
        TerminalWeight=20.0,
    ):
        self.InputWeight = InputWeight
        self.TerminalWeight = TerminalWeight
        self.DesiredPositions = np.asarray(DesiredPositions, dtype=float)
        self.Edges = FrameworkEdges(Graph, self.DesiredPositions)
        self.InitialState = np.asarray(InitialState, dtype=float)
        if self.InitialState.shape != (len(Graph), 2, 2) or SimTime <= 0 or Dt <= 0:
            raise ValueError(
                "Use (AgentNum, 2, 2) initial state and positive horizon/step"
            )
        self.StepNum = int(np.ceil(SimTime / Dt))
        self.Dt = SimTime / self.StepNum
        self.Time = np.linspace(0.0, SimTime, self.StepNum + 1)
        self.SquaredDistances = np.sum(
            (
                self.DesiredPositions[self.Edges[:, 0]]
                - self.DesiredPositions[self.Edges[:, 1]]
            )
            ** 2,
            axis=1,
        )
        self.ReferenceVelocity = np.asarray(ReferenceVelocity, dtype=float)
        self.InputWeight = InputWeight

    def Rollout(self, Control):
        Motion = np.empty((self.StepNum + 1,) + self.InitialState.shape)
        Motion[0] = self.InitialState
        for Index, Input in enumerate(Control):
            Motion[Index + 1, :, 0] = (
                Motion[Index, :, 0]
                + self.Dt * Motion[Index, :, 1]
                + 0.5 * self.Dt**2 * Input
            )
            Motion[Index + 1, :, 1] = Motion[Index, :, 1] + self.Dt * Input
        return Motion

    def StateCost(self, State, TimeValue):
        Position, Velocity = State[:, 0], State[:, 1]
        AgentNum = len(Position)
        PositionError = Position.mean(axis=0) - TimeValue * self.ReferenceVelocity
        VelocityError = Velocity.mean(axis=0) - self.ReferenceVelocity
        VelocityDifference = Velocity - Velocity.mean(axis=0)
        Delta = Position[self.Edges[:, 0]] - Position[self.Edges[:, 1]]
        DistanceError = np.sum(Delta**2, axis=1) - self.SquaredDistances
        Cost = (
            2 * PositionError @ PositionError
            + VelocityError @ VelocityError
            + 0.5 * np.sum(DistanceError**2)
            + 0.5 * np.sum(VelocityDifference**2)
        )
        Gradient = np.zeros_like(State)
        Gradient[:, 0] = 4 * PositionError / AgentNum
        Gradient[:, 1] = 2 * VelocityError / AgentNum + VelocityDifference
        Force = 2 * DistanceError[:, None] * Delta
        np.add.at(Gradient[:, 0], self.Edges[:, 0], Force)
        np.add.at(Gradient[:, 0], self.Edges[:, 1], -Force)
        return Cost, Gradient

    def Objective(self, FlatControl):
        Control = FlatControl.reshape(self.StepNum, len(self.InitialState), 2)
        Motion = self.Rollout(Control)
        Cost, Adjoint = self.StateCost(Motion[-1], self.Time[-1])
        Cost, Adjoint = self.TerminalWeight * Cost, self.TerminalWeight * Adjoint
        Gradient = np.empty_like(Control)
        for Index in range(self.StepNum - 1, -1, -1):
            StageCost, StageGradient = self.StateCost(Motion[Index], self.Time[Index])
            Cost += self.Dt * (
                StageCost + 0.5 * self.InputWeight * np.sum(Control[Index] ** 2)
            )
            Gradient[Index] = (
                self.Dt * self.InputWeight * Control[Index]
                + 0.5 * self.Dt**2 * Adjoint[:, 0]
                + self.Dt * Adjoint[:, 1]
            )
            PreviousAdjoint = Adjoint.copy()
            PreviousAdjoint[:, 1] += self.Dt * Adjoint[:, 0]
            Adjoint = PreviousAdjoint + self.Dt * StageGradient
        return float(Cost), Gradient.ravel()

    def Plan(self):
        InitialControl = np.zeros((self.StepNum, len(self.InitialState), 2))
        InitialCost = self.Objective(InitialControl.ravel())[0]
        Result = minimize(
            self.Objective,
            InitialControl.ravel(),
            jac=True,
            method="L-BFGS-B",
            options={
                "maxiter": 3000,
                "ftol": 1e-9,
                "gtol": 1e-6,
                "maxls": 40,
                "maxcor": 30,
            },
        )
        if not Result.success:
            raise RuntimeError(
                f"Formation optimization did not converge: {Result.message}"
            )
        Control = Result.x.reshape(InitialControl.shape)
        return (
            self.Time,
            self.Rollout(Control),
            Control,
            np.array([InitialCost, Result.fun]),
        )
