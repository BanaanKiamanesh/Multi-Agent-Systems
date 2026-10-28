"""Bearing Formation Maneuvering."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import BearingFormation, BearingManeuver, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
DESIRED = np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]])
ANCHORS = np.array([0, 1])
DRIFT = np.array([0.15, -0.05])
SEED = 73
MODEL_PARAMETERS = {"Gain": 2.0, "Gradient": False}
MODEL_PARAMETERS_2 = {"ProportionalGain": 3.0, "IntegralGain": 1.5}
SIMULATION_PARAMETERS = {"SimTime": 45.0, "Dt": 0.02}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = nx.Graph([(0, 1), (1, 2), (2, 3), (3, 0), (0, 2)])
    Geometry = BearingFormation(Graph, DESIRED, **MODEL_PARAMETERS)
    LeaderVelocity = 0.03 * DESIRED[ANCHORS] + DRIFT
    Model = BearingManeuver(
        Graph,
        Geometry.Bearings,
        ANCHORS,
        DESIRED[ANCHORS],
        LeaderVelocity,
        **MODEL_PARAMETERS_2,
    )
    Initial = np.zeros((4, 2, 2))
    Initial[:, 0] = DESIRED + Random.normal(0, 0.3, DESIRED.shape)
    Initial[ANCHORS, 0] = DESIRED[ANCHORS]
    Time, State = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Motion = State[:, :, 0]
    Targets = (1 + 0.03 * Time[:, None, None]) * DESIRED + Time[:, None, None] * DRIFT
    Metadata.update(
        Kind="targetspatial",
        DesiredPositions=Targets,
        Edges=Geometry.Edges,
        Bearings=Geometry.Bearings,
        AnchorIndices=ANCHORS,
        IntegralState=State[:, :, 1],
    )
    VisualizerInstance = Visualizer(
        "Bearing Formation Maneuvering", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
