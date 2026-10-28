"""Optimal Formation Tracking."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import OptimalFormationPlanner
from plotting import Visualizer

# Edit the parameters below, then run this file.
DESIRED = np.array([[-0.8, -0.8], [0.8, -0.8], [0.8, 0.8], [-0.8, 0.8]])
SEED = 60
MODEL_PARAMETERS = {
    "SimTime": 8.0,
    "Dt": 0.1,
    "ReferenceVelocity": (0.35, -0.15),
    "InputWeight": 0.15,
    "TerminalWeight": 20.0,
}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = nx.Graph([(0, 1), (1, 2), (2, 3), (3, 0), (0, 2)])
    Initial = np.zeros((4, 2, 2))
    Initial[:, 0] = (
        DESIRED + Random.normal(0.0, 0.13, DESIRED.shape) + np.array([-0.7, 0.5])
    )
    Model = OptimalFormationPlanner(Graph, DESIRED, Initial, **MODEL_PARAMETERS)
    Time, Motion, Controls, Objective = Model.Plan()
    Reference = Time[:, None] * Model.ReferenceVelocity
    Metadata.update(
        Kind="optimalformation",
        Edges=Model.Edges,
        Controls=Controls,
        Distances=np.sqrt(Model.SquaredDistances),
        Reference=Reference,
        ReferenceVelocity=Model.ReferenceVelocity,
        Objective=Objective,
    )
    VisualizerInstance = Visualizer(
        "Optimal Formation Tracking", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
