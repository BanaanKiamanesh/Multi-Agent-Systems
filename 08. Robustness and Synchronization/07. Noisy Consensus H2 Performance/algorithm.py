"""Noisy Consensus H2 Performance: mathematical model and numerical routines."""

import networkx as nx
import numpy as np
from scipy.linalg import expm, solve_continuous_lyapunov


def WeightedLaplacian(Graph):
    """Return receiver-row Laplacian and sorted nodes; L @ X is local disagreement."""
    Nodes = list(sorted(Graph.nodes()))
    Weights = nx.to_numpy_array(Graph, nodelist=Nodes, weight="weight", dtype=float)
    if Graph.is_directed():
        Weights = Weights.T
    if not np.all(np.isfinite(Weights)) or np.any(Weights < 0):
        raise ValueError("Communication weights must be finite and nonnegative")
    return np.diag(np.sum(Weights, axis=1)) - Weights, Nodes


def NoisyConsensus(Graph, SimTime=100.0, Dt=0.05, EnsembleNum=128, Seed=92):
    """Exact sampled Ornstein-Uhlenbeck disagreement process and reduced H2 covariance.

    Noise is projected off the mean mode. This avoids simulating an uncontrolled
    Brownian centroid while retaining every disagreement noise direction.
    """
    if (
        Graph.is_directed()
        or not nx.is_connected(Graph)
        or min(SimTime, Dt) <= 0
        or EnsembleNum < 2
    ):
        raise ValueError(
            "Use a connected undirected graph, positive timing and an ensemble"
        )
    Laplacian, _ = WeightedLaplacian(Graph)
    Values, Vectors = np.linalg.eigh(Laplacian)
    if Values[1] <= 1e-10:
        raise ValueError("Positive-weight disagreement dynamics must be stable")
    Basis = Vectors[:, 1:]
    Reduced = -Basis.T @ Laplacian @ Basis
    Covariance = (
        Basis @ solve_continuous_lyapunov(Reduced, -np.eye(len(Graph) - 1)) @ Basis.T
    )
    Time = np.linspace(0.0, SimTime, int(np.ceil(SimTime / Dt)) + 1)
    Step = Time[1] - Time[0]
    Transition = expm(-Step * Laplacian)
    NoiseFactor = Basis * np.sqrt(-np.expm1(-2 * Values[1:] * Step) / (2 * Values[1:]))
    Random = np.random.default_rng(Seed)
    State = np.zeros((EnsembleNum, len(Graph)))
    Motion = [State[0].copy()]
    Variance = [0.0]
    Running = [0.0]
    Sum, Count = 0.0, 0
    for Value in Time[1:]:
        State = (
            State @ Transition.T
            + Random.normal(size=(EnsembleNum, len(Graph) - 1)) @ NoiseFactor.T
        )
        Energy = np.mean(np.sum(State**2, axis=1))
        if Value >= 0.25 * SimTime:
            Sum, Count = Sum + Energy, Count + 1
        Motion.append(State[0].copy())
        Variance.append(Energy)
        Running.append(Sum / Count if Count else Energy)
    return Time, np.array(Motion), Covariance, np.array(Variance), np.array(Running)
