"""Distributed Rigidity Eigenvector: mathematical model and numerical routines."""

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


def MetropolisWeights(Graph):
    """Symmetric stochastic weights 1/(1+max(deg_i,deg_j))."""
    if Graph.is_directed() or not nx.is_connected(Graph):
        raise ValueError("Use a connected undirected graph")
    Nodes = sorted(Graph.nodes())
    Matrix = np.zeros((len(Nodes), len(Nodes)))
    Index = {Node: Number for Number, Node in enumerate(Nodes)}
    for One, Two in Graph.edges():
        if One == Two:
            raise ValueError("Self-loops are unsupported")
        Weight = 1.0 / (1 + max(Graph.degree(One), Graph.degree(Two)))
        Matrix[Index[One], Index[Two]] = Matrix[Index[Two], Index[One]] = Weight
    np.fill_diagonal(Matrix, 1 - Matrix.sum(axis=1))
    return Matrix


def RigidityMatrix(Edges, Position):
    """Jacobian of half the squared edge distances in planar node-major order."""
    Matrix = np.zeros((len(Edges), Position.size))
    for Index, (One, Two) in enumerate(Edges):
        Delta = Position[One] - Position[Two]
        Matrix[Index, 2 * One : 2 * One + 2] = Delta
        Matrix[Index, 2 * Two : 2 * Two + 2] = -Delta
    return Matrix


def DistributedRigidityPower(Graph, Position, Initial, StepNum=300, GossipSteps=70):
    """Deflated shifted power iteration with finite-round neighbor gossip reductions.

    Inputs are a fixed planar framework in a common frame. No eigensolver is
    used. Gauge projection, norm and Rayleigh reductions use Metropolis messages;
    matrix rows of R^T R require at most two-hop edge information.
    """
    Position = np.asarray(Position, dtype=float)
    Edges = FrameworkEdges(Graph, Position)
    Mixing = MetropolisWeights(Graph)
    Matrix = RigidityMatrix(Edges, Position)
    Matrix = Matrix.T @ Matrix
    AgentNum = len(Position)
    if AgentNum < 3 or StepNum < 1 or GossipSteps < 1:
        raise ValueError("Use at least three nodes and positive iteration counts")

    def SumByGossip(Values):
        Values = np.asarray(Values, dtype=float).copy()
        for _ in range(GossipSteps):
            Values = Mixing @ Values
        return AgentNum * Values

    Center = SumByGossip(Position) / AgentNum
    Rotation = np.column_stack([-(Position - Center)[:, 1], (Position - Center)[:, 0]])
    RotationNorm = np.sqrt(SumByGossip(np.sum(Rotation**2, axis=1)))
    if np.min(RotationNorm) < 1e-10:
        raise ValueError("Degenerate framework")
    Rotation /= RotationNorm[:, None]
    Gauge = np.zeros((AgentNum, 2, 3))
    Gauge[:, 0, 0], Gauge[:, 1, 1] = 1 / np.sqrt(AgentNum), 1 / np.sqrt(AgentNum)
    Gauge[:, :, 2] = Rotation
    Alpha = 0.8 / max(np.linalg.norm(Matrix, ord=np.inf), 1e-10)
    Vector = np.array(Initial, dtype=float, copy=True)
    if Vector.shape != Position.shape:
        raise ValueError("Initial vector must have the framework shape")
    Motion, Estimates = [], []
    for _ in range(StepNum + 1):
        Coefficients = SumByGossip(np.einsum("ndk,nd->nk", Gauge, Vector))
        Vector -= np.einsum("ndk,nk->nd", Gauge, Coefficients)
        Norm = np.sqrt(SumByGossip(np.sum(Vector**2, axis=1)))
        if np.min(Norm) < 1e-12:
            raise ValueError("Initial vector lies in the rigid-motion nullspace")
        Vector /= Norm[:, None]
        Product = (Matrix @ Vector.ravel()).reshape(Position.shape)
        Rayleigh = SumByGossip(np.sum(Vector * Product, axis=1))
        Motion.append(Vector.copy())
        Estimates.append(Rayleigh)
        Vector -= Alpha * Product
    return np.arange(StepNum + 1, dtype=float), np.array(Motion), np.array(Estimates)


def RigidityEigenpair(Edges, Position):
    Matrix = RigidityMatrix(Edges, Position)
    Values, Vectors = np.linalg.eigh(Matrix.T @ Matrix)
    # Three rigid-body modes in the plane: two translations and one rotation.
    return float(Values[3]), Vectors[:, 3].reshape(Position.shape)
