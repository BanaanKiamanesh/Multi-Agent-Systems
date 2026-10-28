"""Heterogeneous Fractional Network: mathematical model and numerical routines."""

import networkx as nx
import numpy as np


class HeterogeneousFractionalNetwork:
    """Seven nonidentical fractional agents with time-varying intrinsic dynamics.

    Inspired by fracsystem2.m: directed asymmetric coupling, one signed
    antagonistic interaction, and seven potentially distinct orders.
    """

    def __init__(self, Gain=1.6):

        Graph = nx.DiGraph()
        Graph.add_nodes_from(range(7))
        Edges = [
            (1, 0, 2.1),
            (3, 0, 0.7),
            (6, 0, 0.6),
            (0, 1, 1.5),
            (2, 1, 0.8),
            (3, 1, 0.8),
            (6, 1, 0.6),
            (0, 2, 1.5),
            (6, 3, 1.5),
            (1, 3, 0.8),
            (0, 3, 0.8),
            (3, 4, 1.6),
            (4, 5, 1.6),
            (5, 6, 0.8),
            (3, 6, 1.6),
            (0, 6, 0.8),
            (1, 6, -0.12),
        ]
        Graph.add_weighted_edges_from(Edges)
        self.Graph = Graph
        self.Weight = nx.to_numpy_array(Graph, nodelist=range(7), weight="weight").T
        self.Gain = Gain

    def SysDyn(self, TimeValue, State):
        Difference = State[None, :] - State[:, None]
        Interaction = np.sum(self.Weight * Difference, axis=1)
        Intrinsic = (0.18 * np.sin(0.6 * TimeValue) - 0.33) * State
        return Intrinsic + self.Gain * Interaction


def TimeGrid(SimTime, Dt):
    """Uniform samples ending exactly at SimTime, with step no larger than Dt."""
    if not np.isfinite(SimTime) or not np.isfinite(Dt) or min(SimTime, Dt) <= 0:
        raise ValueError("SimTime and Dt must be finite and positive")
    return np.linspace(0.0, SimTime, int(np.ceil(SimTime / Dt)) + 1)


def SimulateFractional(Dynamics, InitialState, Order=0.85, SimTime=14, Dt=0.04):
    """Explicit Grunwald-Letnikov approximation of Caputo D^Order(X) = F(X).

    Uses the initialized GL derivative of X-X0; for 0 < Order <= 1 and
    sufficiently smooth solutions it approximates the Caputo derivative.
    Computational cost is quadratic in the time-grid size.
    """
    if (
        not np.all(np.isfinite(Order))
        or np.any(np.asarray(Order) <= 0)
        or np.any(np.asarray(Order) > 1)
    ):
        raise ValueError("Fractional order must be in (0, 1]")
    Time = TimeGrid(SimTime, Dt)
    Dt = Time[1] - Time[0]
    InitialState = np.asarray(InitialState, dtype=float)
    if not np.all(np.isfinite(InitialState)):
        raise ValueError("InitialState must be finite")
    State = np.empty((len(Time),) + np.shape(InitialState), dtype=float)
    State[0] = InitialState
    Orders = np.broadcast_to(np.asarray(Order, dtype=float), np.shape(InitialState))
    Coefficients = np.empty((len(Time),) + np.shape(InitialState), dtype=float)
    Coefficients[0] = 1.0
    for Index in range(1, len(Time)):
        Coefficients[Index] = Coefficients[Index - 1] * (1.0 - (Orders + 1.0) / Index)
    for Index in range(1, len(Time)):
        Memory = np.sum(
            Coefficients[1 : Index + 1] * (State[Index - 1 :: -1] - InitialState),
            axis=0,
        )
        State[Index] = (
            InitialState
            + Dt**Orders * Dynamics(Time[Index - 1], State[Index - 1])
            - Memory
        )
        if not np.all(np.isfinite(State[Index])):
            raise FloatingPointError("Fractional integration diverged")
    return Time, State
