"""Proximal Relative Estimation: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


def FormationOffsets(AgentNum=5, Radius=1.4):
    Angle = np.arange(AgentNum) * 2.0 * np.pi / AgentNum
    return Radius * np.column_stack([np.cos(Angle), np.sin(Angle)])


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


def WeightedLaplacian(Graph):
    """Return receiver-row Laplacian and sorted nodes; L @ X is local disagreement."""
    Nodes = list(sorted(Graph.nodes()))
    Weights = nx.to_numpy_array(Graph, nodelist=Nodes, weight="weight", dtype=float)
    if Graph.is_directed():
        Weights = Weights.T
    if not np.all(np.isfinite(Weights)) or np.any(Weights < 0):
        raise ValueError("Communication weights must be finite and nonnegative")
    return np.diag(np.sum(Weights, axis=1)) - Weights, Nodes


def SimulateProximalEstimation(
    Graph, Measurements, Initial, AnchorPosition, Rho=2.0, StepNum=200
):
    """Anchored local PP/Jacobi update (Fabris et al., Eq. 8), planar coordinates.

    Each edge measurement is p[j]-p[i]. The diagonal regularization is local;
    this routine does not invert a global Laplacian at every step.
    """
    Initial = np.asarray(Initial, dtype=float)
    Edges = FrameworkEdges(Graph, Initial)
    Laplacian, _ = WeightedLaplacian(Graph)
    if not np.allclose(
        nx.to_numpy_array(Graph, nodelist=sorted(Graph)),
        nx.to_numpy_array(Graph, nodelist=sorted(Graph), weight=None),
    ):
        raise ValueError("This PP update uses unit edge weights")
    Measurements = np.asarray(Measurements, dtype=float)
    if Measurements.shape != (len(Edges), 2) or Rho <= 0 or StepNum < 1:
        raise ValueError(
            "Provide planar edge measurements and positive regularization/iterations"
        )
    Degree = np.diag(Laplacian)
    Adjacency = np.diag(Degree) - Laplacian
    Forcing = np.zeros_like(Initial)
    np.add.at(Forcing, Edges[:, 0], -Measurements)
    np.add.at(Forcing, Edges[:, 1], Measurements)
    Motion = np.empty((StepNum + 1,) + Initial.shape)
    Motion[0] = Initial
    Motion[0, 0] = AnchorPosition
    for Index in range(StepNum):
        Motion[Index + 1] = (
            2 * Adjacency @ Motion[Index] + Rho * Motion[Index] + 2 * Forcing
        ) / (2 * Degree + Rho)[:, None]
        Motion[Index + 1, 0] = AnchorPosition
    return np.arange(StepNum + 1, dtype=float), Motion
