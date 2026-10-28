"""Henneberg Rigid Formation."""

from pathlib import Path

import numpy as np
from algorithm import DistanceBasedFormation, HennebergGraph, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
DESIRED = np.array(
    [
        [-1.49, -0.71],
        [0.0, -0.25],
        [-0.52, -1.47],
        [-0.01, 0.95],
        [0.94, -1.24],
        [-0.76, -0.17],
    ]
)
SEED = 77
MODEL_PARAMETERS = {"Gain": 2.0, "DistanceFloor": 1e-08}
SIMULATION_PARAMETERS = {"SimTime": 65.0, "Dt": 0.025}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = HennebergGraph([("I", 0, 1), ("I", 0, 2), ("II", 1, 2, 3), ("I", 0, 4)])
    Model = DistanceBasedFormation(Graph, DESIRED, **MODEL_PARAMETERS)
    Initial = DESIRED + Random.normal(0, 0.1, DESIRED.shape) + [0.6, -0.4]
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(
        Kind="distanceformation",
        Edges=Model.Edges,
        Distances=Model.Distances,
        DesiredPositions=DESIRED,
    )
    VisualizerInstance = Visualizer("Henneberg Rigid Formation", Time, Motion, Metadata)
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
