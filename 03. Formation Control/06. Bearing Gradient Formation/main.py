"""Bearing Gradient Formation."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import BearingFormation, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
DESIRED = np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]])
INITIAL = np.array([[0.2, 1.0], [-0.2, 0.5], [0.2, -0.5], [-0.2, -1.0]])
MODEL_PARAMETERS = {"Gain": 3.0, "Gradient": True}
SIMULATION_PARAMETERS = {"SimTime": 35.0, "Dt": 0.005}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Metadata = {}
    Graph = nx.Graph([(0, 1), (1, 2), (2, 3), (3, 0), (0, 2)])
    Model = BearingFormation(Graph, DESIRED, **MODEL_PARAMETERS)
    Time, Motion = Simulate(Model.SysDyn, INITIAL, **SIMULATION_PARAMETERS)
    Metadata.update(
        Kind="bearingformation",
        Edges=Model.Edges,
        Bearings=Model.Bearings,
        DesiredPositions=DESIRED,
    )
    VisualizerInstance = Visualizer(
        "Bearing Gradient Formation", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
