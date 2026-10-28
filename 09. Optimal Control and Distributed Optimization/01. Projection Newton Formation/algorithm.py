"""Projection Newton Formation: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


def WeightedLaplacian(Graph):
    """Return receiver-row Laplacian and sorted nodes; L @ X is local disagreement."""
    Nodes = list(sorted(Graph.nodes()))
    Weights = nx.to_numpy_array(Graph, nodelist=Nodes, weight="weight", dtype=float)
    if Graph.is_directed():
        Weights = Weights.T
    if not np.all(np.isfinite(Weights)) or np.any(Weights < 0):
        raise ValueError("Communication weights must be finite and nonnegative")
    return np.diag(np.sum(Weights, axis=1)) - Weights, Nodes


class ProjectionNewtonFormation:
    """Feedback projection, Riccati Newton direction and Armijo update.

    The plant is a planar double-integrator network with quadratic displacement,
    centroid and velocity costs. The LQ specialization has an exact Newton step;
    this does not reproduce nonlinear PRONTO's second-order projection terms.
    """

    def __init__(
        self,
        Graph,
        Offsets,
        InitialState,
        SimTime=6.0,
        Dt=0.1,
        ReferenceVelocity=(0.3, -0.1),
    ):
        if Graph.is_directed() or not nx.is_connected(Graph):
            raise ValueError(
                "The quadratic formation cost requires a connected undirected graph"
            )
        self.Offsets = np.asarray(Offsets, dtype=float)
        self.AgentNum = len(Graph)
        self.InitialState = np.asarray(InitialState, dtype=float)
        if (
            self.Offsets.shape != (self.AgentNum, 2)
            or self.InitialState.shape != (self.AgentNum, 2, 2)
            or SimTime <= 0
            or Dt <= 0
        ):
            raise ValueError(
                "Use planar offsets, (AgentNum,2,2) states and positive timing"
            )
        self.StepNum = int(np.ceil(SimTime / Dt))
        self.Dt = SimTime / self.StepNum
        self.Time = np.linspace(0.0, SimTime, self.StepNum + 1)
        Size = 2 * self.AgentNum
        Identity = np.eye(Size)
        self.A = np.block(
            [[Identity, self.Dt * Identity], [np.zeros_like(Identity), Identity]]
        )
        self.B = np.vstack([0.5 * self.Dt**2 * Identity, self.Dt * Identity])
        self.TrackingGain = np.hstack([1.2 * Identity, 1.6 * Identity])
        if np.max(np.abs(np.linalg.eigvals(self.A - self.B @ self.TrackingGain))) >= 1:
            raise ValueError("Time step is too large for the projection tracking gain")
        Laplacian, _ = WeightedLaplacian(Graph)
        Centroid = np.ones_like(Laplacian) / self.AgentNum
        PositionWeight = np.kron(2 * Laplacian + Centroid, np.eye(2))
        self.Q = np.block(
            [
                [PositionWeight, np.zeros_like(Identity)],
                [np.zeros_like(Identity), Identity],
            ]
        )
        self.R = 0.2 * Identity
        self.TerminalWeight = 10 * self.Q
        self.ReferenceVelocity = np.asarray(ReferenceVelocity, dtype=float)
        self.Reference = np.array(
            [
                np.concatenate(
                    [
                        (self.Offsets + Value * self.ReferenceVelocity).ravel(),
                        np.tile(self.ReferenceVelocity, self.AgentNum),
                    ]
                )
                for Value in self.Time
            ]
        )
        self.InitialVector = np.concatenate(
            [self.InitialState[:, 0].ravel(), self.InitialState[:, 1].ravel()]
        )

    def Project(self, StateCurve, InputCurve):
        """P(alpha,mu): u=mu+K(alpha-x), x+=A*x+B*u, x[0]=x0."""
        if StateCurve.shape != self.Reference.shape or InputCurve.shape != (
            self.StepNum,
            2 * self.AgentNum,
        ):
            raise ValueError("Projection curves have incompatible shapes")
        State = np.empty_like(StateCurve)
        Control = np.empty_like(InputCurve)
        State[0] = self.InitialVector
        for Index in range(self.StepNum):
            Control[Index] = InputCurve[Index] + self.TrackingGain @ (
                StateCurve[Index] - State[Index]
            )
            State[Index + 1] = self.A @ State[Index] + self.B @ Control[Index]
        return State, Control

    def Cost(self, State, Control):
        Error = State - self.Reference
        return float(
            0.5
            * self.Dt
            * (
                np.einsum("ti,ij,tj->", Error[:-1], self.Q, Error[:-1])
                + np.einsum("ti,ij,tj->", Control, self.R, Control)
            )
            + 0.5 * Error[-1] @ self.TerminalWeight @ Error[-1]
        )

    def Plan(self):
        State, Control = self.Project(
            self.Reference, np.zeros((self.StepNum, 2 * self.AgentNum))
        )
        InitialCost = self.Cost(State, Control)
        # The tangent Newton subproblem is an affine-initial-state LQR. Reference
        # motion is feasible (constant velocity), so error coordinates are homogeneous.
        Value = self.TerminalWeight.copy()
        Gains = []
        for _ in range(self.StepNum):
            Gain = np.linalg.solve(
                self.Dt * self.R + self.B.T @ Value @ self.B, self.B.T @ Value @ self.A
            )
            Value = self.Dt * self.Q + self.A.T @ Value @ (self.A - self.B @ Gain)
            Value = 0.5 * (Value + Value.T)
            Gains.append(Gain)
        Error = self.InitialVector - self.Reference[0]
        OptimalState, OptimalControl = [self.InitialVector], []
        for Index, Gain in enumerate(reversed(Gains)):
            Input = -Gain @ Error
            Error = self.A @ Error + self.B @ Input
            OptimalState.append(Error + self.Reference[Index + 1])
            OptimalControl.append(Input)
        DirectionState = np.array(OptimalState) - State
        DirectionControl = np.array(OptimalControl) - Control
        Error = State - self.Reference
        Slope = (
            self.Dt
            * (
                np.einsum("ti,ij,tj->", Error[:-1], self.Q, DirectionState[:-1])
                + np.einsum("ti,ij,tj->", Control, self.R, DirectionControl)
            )
            + Error[-1] @ self.TerminalWeight @ DirectionState[-1]
        )
        Scale = 1.0
        for _ in range(30):
            TrialState, TrialControl = self.Project(
                State + Scale * DirectionState, Control + Scale * DirectionControl
            )
            Cost = self.Cost(TrialState, TrialControl)
            if Cost <= InitialCost + 1e-4 * Scale * Slope + 1e-12:
                break
            Scale *= 0.5
        else:
            raise RuntimeError("Projection Newton line search failed")
        Position = TrialState[:, : 2 * self.AgentNum].reshape(-1, self.AgentNum, 2)
        Velocity = TrialState[:, 2 * self.AgentNum :].reshape(Position.shape)
        Motion = np.stack([Position, Velocity], axis=2)
        return (
            self.Time,
            Motion,
            TrialControl.reshape(-1, self.AgentNum, 2),
            np.array([InitialCost, Cost]),
        )
