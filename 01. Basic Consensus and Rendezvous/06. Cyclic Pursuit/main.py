"""Cyclic Pursuit."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import FirstOrderConsensus, Simulate, WeightedLaplacian
from plotting import Visualizer

# Edit the parameters below, then run this file.
INITIAL = np.array(
    [
        [0.0, 1.0],
        [-0.95106, 0.30902],
        [-0.58779, -0.80902],
        [0.58779, -0.80902],
        [0.95106, 0.30902],
    ]
)
MODEL_PARAMETERS = {"Gain": 1.0}
SIMULATION_PARAMETERS = {"SimTime": 12.0, "Dt": 0.025}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Metadata = {}
    Graph = nx.DiGraph([((Index + 1) % 5, Index) for Index in range(5)])
    Laplacian, _ = WeightedLaplacian(Graph)
    Model = FirstOrderConsensus(Laplacian, **MODEL_PARAMETERS)
    Time, Motion = Simulate(Model.SysDyn, INITIAL, **SIMULATION_PARAMETERS)
    Metadata.update(Kind="spatial", Laplacian=Laplacian)
    VisualizerInstance = Visualizer("Cyclic Pursuit", Time, Motion, Metadata)
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
