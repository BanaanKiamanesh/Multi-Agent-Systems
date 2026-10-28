"""Range-Only Frame Estimation: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


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


def RigidityMatrix(Edges, Position):
    """Jacobian of half the squared edge distances in planar node-major order."""
    Matrix = np.zeros((len(Edges), Position.size))
    for Index, (One, Two) in enumerate(Edges):
        Delta = Position[One] - Position[Two]
        Matrix[Index, 2 * One : 2 * One + 2] = Delta
        Matrix[Index, 2 * Two : 2 * Two + 2] = -Delta
    return Matrix


class RangeFrameEstimator:
    """Local gradient reconstruction from squared ranges and three oriented anchors.

    Pinning three noncollinear known positions fixes translation, rotation and
    reflection. Other nodes exchange estimates with their range neighbors only.
    """

    def __init__(self, Graph, Ranges, AnchorIndices, AnchorPositions, Gain=0.6):
        self.Edges = FrameworkEdges(Graph, np.zeros((len(Graph), 2)))
        self.Ranges = np.asarray(Ranges, dtype=float)
        self.AnchorIndices = np.asarray(AnchorIndices, dtype=int)
        self.AnchorPositions = np.asarray(AnchorPositions, dtype=float)
        if (
            self.Ranges.shape != (len(self.Edges),)
            or np.any(self.Ranges <= 0)
            or len(self.AnchorIndices) < 3
            or len(np.unique(self.AnchorIndices)) != len(self.AnchorIndices)
            or np.any(self.AnchorIndices < 0)
            or np.any(self.AnchorIndices >= len(Graph))
            or self.AnchorPositions.shape != (len(self.AnchorIndices), 2)
            or Gain <= 0
        ):
            raise ValueError(
                "Use positive edge ranges and at least three valid planar anchors"
            )
        if (
            np.linalg.matrix_rank(
                self.AnchorPositions - self.AnchorPositions.mean(axis=0)
            )
            < 2
        ):
            raise ValueError("Anchors must not be collinear")
        self.Gain = Gain

    def ResidualJacobian(self, Position):
        Position = np.array(Position, dtype=float, copy=True)
        Position[self.AnchorIndices] = self.AnchorPositions
        Delta = Position[self.Edges[:, 0]] - Position[self.Edges[:, 1]]
        Residual = 0.5 * (np.sum(Delta**2, axis=1) - self.Ranges**2)
        return Residual, RigidityMatrix(self.Edges, Position)

    def SysDyn(self, TimeValue, Position):
        Residual, Jacobian = self.ResidualJacobian(Position)
        Rate = -self.Gain * (Jacobian.T @ Residual).reshape(Position.shape)
        Rate[self.AnchorIndices] = 0.0
        return Rate

    def GaussNewton(self, Initial, StepNum=25, Damping=1e-5, InnerSteps=200):
        """Damped GN with two-hop block-Jacobi normal-equation iterations.

        A coordinated backtracking step checks the global residual. This is not
        the reference paper's fully decentralized Hessian consensus solver.
        """
        if Damping <= 0 or InnerSteps < 1 or StepNum < 1:
            raise ValueError("Positive damping and iteration counts are required")
        Position = np.array(Initial, dtype=float, copy=True)
        Position[self.AnchorIndices] = self.AnchorPositions
        Followers = np.setdiff1d(np.arange(len(Position)), self.AnchorIndices)
        if not len(Followers):
            raise ValueError("Specify at least one unanchored node")
        Dofs = (2 * Followers[:, None] + np.arange(2)).ravel()
        Motion = [Position.copy()]
        for _ in range(StepNum):
            Residual, Jacobian = self.ResidualJacobian(Position)
            Reduced = Jacobian[:, Dofs]
            Normal = Reduced.T @ Reduced + Damping * np.eye(len(Dofs))
            Right = -Reduced.T @ Residual
            # Under-relaxed block Jacobi: every product uses normal-matrix neighbors.
            Inverse = [
                np.linalg.inv(Normal[2 * i : 2 * i + 2, 2 * i : 2 * i + 2])
                for i in range(len(Followers))
            ]
            Direction = np.zeros(len(Dofs))
            # A spectral bound makes the relaxation valid even for dense frameworks.
            Preconditioned = np.vstack(
                [Block @ Normal[2 * i : 2 * i + 2] for i, Block in enumerate(Inverse)]
            )
            Relaxation = 0.9 / max(np.linalg.norm(Preconditioned, ord=np.inf), 1.0)
            for _ in range(InnerSteps):
                Error = Right - Normal @ Direction
                Direction += Relaxation * np.concatenate(
                    [
                        Block @ Error[2 * i : 2 * i + 2]
                        for i, Block in enumerate(Inverse)
                    ]
                )
            Scale = 1.0
            for _ in range(25):
                Trial = Position.copy()
                Trial[Followers] += Scale * Direction.reshape(-1, 2)
                if (
                    np.linalg.norm(self.ResidualJacobian(Trial)[0])
                    <= np.linalg.norm(Residual) + 1e-14
                ):
                    Position = Trial
                    break
                Scale *= 0.5
            else:
                raise RuntimeError("Range estimator line search failed")
            Motion.append(Position.copy())
        return np.arange(len(Motion), dtype=float), np.array(Motion)


def FormationOffsets(AgentNum=5, Radius=1.4):
    Angle = np.arange(AgentNum) * 2.0 * np.pi / AgentNum
    return Radius * np.column_stack([np.cos(Angle), np.sin(Angle)])


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
