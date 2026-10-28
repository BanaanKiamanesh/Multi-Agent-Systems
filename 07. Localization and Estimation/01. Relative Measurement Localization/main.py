"""Relative Measurement Localization."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import FormationOffsets, RelativeMeasurementEstimator, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
SEED = 55
FORMATION_OFFSETS_PARAMETERS = {"AgentNum": 5, "Radius": 1.6}
MODEL_PARAMETERS = {"AnchorIndex": 0, "Gain": 1.5}
SIMULATION_PARAMETERS = {"SimTime": 24.0, "Dt": 0.025}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = nx.wheel_graph(6)
    Desired = np.vstack([np.zeros(2), FormationOffsets(**FORMATION_OFFSETS_PARAMETERS)])
    Edges = np.array(list(Graph.edges()), dtype=int)
    Measurements = Desired[Edges[:, 1]] - Desired[Edges[:, 0]]
    Model = RelativeMeasurementEstimator(
        Graph, Measurements, Desired[0], **MODEL_PARAMETERS
    )
    Initial = Random.uniform(-3, 3, Desired.shape)
    Initial[0] = Desired[0]
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(
        Kind="targetspatial",
        DesiredPositions=Desired,
        Edges=Edges,
        Measurements=Measurements,
        AnchorIndices=np.array([0]),
    )
    VisualizerInstance = Visualizer(
        "Relative Measurement Localization", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
