"""Projection Newton Formation."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import ProjectionNewtonFormation
from plotting import Visualizer

# Edit the parameters below, then run this file.
DESIRED = np.array([[-0.8, -0.8], [0.8, -0.8], [0.8, 0.8], [-0.8, 0.8]])
SEED = 75
MODEL_PARAMETERS = {"SimTime": 6.0, "Dt": 0.1, "ReferenceVelocity": (0.3, -0.1)}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = nx.cycle_graph(4)
    Initial = np.zeros((4, 2, 2))
    Initial[:, 0] = DESIRED + Random.normal(0, 0.2, DESIRED.shape) + [-0.7, 0.4]
    Model = ProjectionNewtonFormation(Graph, DESIRED, Initial, **MODEL_PARAMETERS)
    Time, Motion, Controls, Objective = Model.Plan()
    Edges = np.array(list(Graph.edges()))
    Distances = np.linalg.norm(DESIRED[Edges[:, 0]] - DESIRED[Edges[:, 1]], axis=1)
    Metadata.update(
        Kind="optimalformation",
        Edges=Edges,
        Distances=Distances,
        Reference=Time[:, None] * Model.ReferenceVelocity,
        ReferenceVelocity=Model.ReferenceVelocity,
        Controls=Controls,
        Objective=Objective,
    )
    VisualizerInstance = Visualizer(
        "Projection Newton Formation", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
