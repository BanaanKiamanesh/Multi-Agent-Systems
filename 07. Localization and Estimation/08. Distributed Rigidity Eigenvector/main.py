"""Distributed Rigidity Eigenvector."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import DistributedRigidityPower, RigidityEigenpair
from plotting import Visualizer

# Edit the parameters below, then run this file.
POSITION = np.array([[-1.0, -0.8], [1.0, -0.6], [1.2, 0.9], [-0.9, 1.0]])
SEED = 70
SIMULATION_PARAMETERS = {"StepNum": 800, "GossipSteps": 70}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = nx.Graph([(0, 1), (1, 2), (2, 3), (3, 0), (0, 2)])
    Initial = Random.normal(size=POSITION.shape)
    Time, Motion, Estimates = DistributedRigidityPower(
        Graph, POSITION, Initial, **SIMULATION_PARAMETERS
    )
    Value, _ = RigidityEigenpair(np.array(list(Graph.edges())), POSITION)
    Metadata.update(
        Kind="eigenestimate",
        Eigenvalue=np.array([Value]),
        EigenvalueEstimates=Estimates,
        DesiredPositions=POSITION,
        TimeLabel="Power iteration",
    )
    VisualizerInstance = Visualizer(
        "Distributed Rigidity Eigenvector", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
