"""Distance-Based Formation."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import DistanceBasedFormation, FormationOffsets, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
AGENT_NUM = 6
ROTATION_ANGLE = 0.25
SEED = 49
FORMATION_OFFSETS_PARAMETERS = {"AgentNum": 5, "Radius": 1.6}
MODEL_PARAMETERS = {"Gain": 1.8, "DistanceFloor": 1e-08}
SIMULATION_PARAMETERS = {"SimTime": 11.0, "Dt": 0.025}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Offsets = np.vstack([np.zeros(2), FormationOffsets(**FORMATION_OFFSETS_PARAMETERS)])
    Graph = nx.wheel_graph(AGENT_NUM)
    Model = DistanceBasedFormation(Graph, Offsets, **MODEL_PARAMETERS)
    Rotation = np.array(
        [
            [np.cos(ROTATION_ANGLE), -np.sin(ROTATION_ANGLE)],
            [np.sin(ROTATION_ANGLE), np.cos(ROTATION_ANGLE)],
        ]
    )
    Initial = (
        Offsets @ Rotation.T
        + Random.normal(0, 0.32, Offsets.shape)
        + np.array([1.2, -0.7])
    )
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(
        Kind="distanceformation",
        Edges=Model.Edges,
        Distances=Model.Distances,
        DesiredPositions=Offsets,
    )
    VisualizerInstance = Visualizer("Distance-Based Formation", Time, Motion, Metadata)
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
