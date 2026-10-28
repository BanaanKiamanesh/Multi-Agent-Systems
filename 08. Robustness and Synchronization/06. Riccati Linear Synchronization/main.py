"""Riccati Linear Synchronization."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import RiccatiSynchronization, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
SEED = 82
MODEL_PARAMETERS = {"Coupling": None}
SIMULATION_PARAMETERS = {"SimTime": 38.0, "Dt": 0.025}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = nx.cycle_graph(6)
    A, B = (np.array([[0.0, 1.0], [-1.0, 0.0]]), np.array([[0.0], [1.0]]))
    Model = RiccatiSynchronization(Graph, A, B, **MODEL_PARAMETERS)
    Initial = Random.uniform(-2, 2, (6, 2))
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(
        Kind="linear",
        A=A,
        B=B,
        Feedback=Model.K,
        Riccati=Model.P,
        Coupling=np.array([Model.Coupling]),
        Laplacian=Model.Laplacian,
    )
    VisualizerInstance = Visualizer(
        "Riccati Linear Synchronization", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
