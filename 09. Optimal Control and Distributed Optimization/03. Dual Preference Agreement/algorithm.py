"""Dual Preference Agreement: mathematical model and numerical routines."""

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


class PreferenceAgreement:
    """Finite-horizon quadratic preferences with neighbor dual ascent, Padua conclusion pp.124-151.

    Local integrator plans minimize position preferences and input energy, with
    a terminal agreement constraint enforced by node multipliers in range(L).
    """

    def __init__(self, Graph, Preferences, StateWeights, InputWeights):
        if Graph.is_directed() or not nx.is_connected(Graph):
            raise ValueError("Use a connected undirected graph")
        self.Laplacian, _ = WeightedLaplacian(Graph)
        self.Preferences = np.asarray(Preferences, dtype=float)
        self.StateWeights, self.InputWeights = (
            np.asarray(StateWeights, dtype=float),
            np.asarray(InputWeights, dtype=float),
        )
        if (
            self.Preferences.shape != (len(Graph),)
            or self.StateWeights.shape != self.Preferences.shape
            or self.InputWeights.shape != self.Preferences.shape
            or min(self.StateWeights.min(), self.InputWeights.min()) <= 0
        ):
            raise ValueError(
                "Provide one preference and positive state/input weights per node"
            )
        self.MaximumEigenvalue = np.linalg.eigvalsh(self.Laplacian)[-1]
        self.Cache = {}

    def Factors(self, Horizon):
        if Horizon not in self.Cache:
            Cumulative = np.tril(np.ones((Horizon, Horizon)))
            Inverse = np.array(
                [
                    np.linalg.inv(Q * Cumulative.T @ Cumulative + R * np.eye(Horizon))
                    for Q, R in zip(self.StateWeights, self.InputWeights)
                ]
            )
            Sensitivity = np.einsum("nij->n", Inverse)
            self.Cache[Horizon] = (
                Cumulative,
                Inverse,
                0.9 / (Sensitivity.max() * self.MaximumEigenvalue),
            )
        return self.Cache[Horizon]

    def LocalPlans(self, Initial, Multipliers, Horizon):
        Cumulative, Inverse, Step = self.Factors(Horizon)
        Right = (
            -self.StateWeights[:, None]
            * (Initial - self.Preferences)[:, None]
            * Cumulative.sum(axis=0)
            - Multipliers[:, None]
        )
        Control = np.einsum("nij,nj->ni", Inverse, Right)
        Motion = Initial[:, None] + Control @ Cumulative.T
        return Motion, Control, Step

    def Solve(self, Initial, Horizon=15, Iterations=800):
        if Horizon < 1 or Iterations < 1:
            raise ValueError("Use positive horizon and iteration count")
        Multipliers = np.zeros(len(self.Preferences))
        History = []
        for _ in range(Iterations):
            Motion, Control, Step = self.LocalPlans(Initial, Multipliers, Horizon)
            History.append(np.std(Motion[:, -1]))
            Multipliers += Step * self.Laplacian @ Motion[:, -1]
        Motion, Control, _ = self.LocalPlans(Initial, Multipliers, Horizon)
        return np.vstack([Initial, Motion.T]), Control.T, Multipliers, np.array(History)

    def ShrinkingHorizon(self, Initial, Horizon=30, Sweeps=1):
        if Horizon < 1 or Sweeps < 1:
            raise ValueError("Use positive horizon and communication budget")
        State = np.asarray(Initial, dtype=float).copy()
        Motion, Inputs, PredictionError = [State.copy()], [], []
        Multipliers = np.zeros(len(State))
        for Remaining in range(Horizon, 0, -1):
            for _ in range(Sweeps):
                Plan, Control, Step = self.LocalPlans(State, Multipliers, Remaining)
                Multipliers += Step * self.Laplacian @ Plan[:, -1]
            # Replan after communication, then execute only the first action.
            Plan, Control, _ = self.LocalPlans(State, Multipliers, Remaining)
            PredictionError.append(np.std(Plan[:, -1]))
            State += Control[:, 0]
            Motion.append(State.copy())
            Inputs.append(Control[:, 0])
        return (
            np.array(Motion),
            np.array(Inputs),
            np.array(PredictionError),
            Multipliers,
        )

    def Cost(self, Motion, Controls):
        return 0.5 * np.sum(
            (Motion[1:] - self.Preferences) ** 2 * self.StateWeights
            + Controls**2 * self.InputWeights
        )
