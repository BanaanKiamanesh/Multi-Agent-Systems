"""Bearing-Based Localization."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import AnchoredBearingNetwork, BearingFormation, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
DESIRED = np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]])
ANCHORS = np.array([0, 1])
SEED = 63
MODEL_PARAMETERS = {"Gain": 3.0, "Gradient": False}
MODEL_PARAMETERS_2 = {"Gain": 3.0}
SIMULATION_PARAMETERS = {"SimTime": 45.0, "Dt": 0.025}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = nx.Graph([(0, 1), (1, 2), (2, 3), (3, 0), (0, 2)])
    Model = BearingFormation(Graph, DESIRED, **MODEL_PARAMETERS)
    Anchored = AnchoredBearingNetwork(
        Graph, Model.Bearings, ANCHORS, DESIRED[ANCHORS], **MODEL_PARAMETERS_2
    )
    Initial = Random.uniform(-3.0, 3.0, DESIRED.shape)
    Initial[ANCHORS] = DESIRED[ANCHORS]
    Time, Motion = Simulate(Anchored.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(
        Kind="targetspatial",
        Edges=Model.Edges,
        Bearings=Model.Bearings,
        DesiredPositions=DESIRED,
        AnchorIndices=ANCHORS,
        BearingMatrix=Anchored.Matrix,
    )
    VisualizerInstance = Visualizer(
        "Bearing-Based Localization", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
