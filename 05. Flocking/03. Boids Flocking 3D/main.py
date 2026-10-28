"""Boids Flocking 3D."""

from pathlib import Path

import numpy as np
from algorithm import BoidsController, FlockDiagnostics, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
SEED = 140
MODEL_PARAMETERS = {
    "Radius": 2.5,
    "SeparationGain": 0.16,
    "AlignmentGain": 0.6,
    "CohesionGain": 0.25,
    "GoalGain": 0.5,
}
SIMULATION_PARAMETERS = {"SimTime": 72.0, "Dt": 0.02}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def BoidsReference(TimeValue):
    Goals = np.array(
        [
            [1.0, 0.0, 0.3],
            [0.0, 1.0, 0.6],
            [-1.0, 0.0, 0.3],
            [0.0, -1.0, 0.6],
            [1.0, 0.0, 0.3],
        ]
    )
    Index = min(int(TimeValue // 12.0), len(Goals) - 1)
    return (Goals[Index], np.zeros(3))


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Model = BoidsController(BoidsReference, **MODEL_PARAMETERS)
    Initial = np.zeros((8, 2, 3))
    Initial[:, 0] = np.array(
        [[x, y, z] for x in (-0.45, 0.45) for y in (-0.45, 0.45) for z in (-0.45, 0.45)]
    )
    Initial[:, 0] += Random.normal(0, 0.03, (8, 3)) + [-1.5, -0.5, 0.4]
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Reference = np.array([BoidsReference(Value)[0] for Value in Time])
    Rates = np.array([BoidsReference(Value)[1] for Value in Time])
    Metadata = dict(
        Kind="archiveflocking",
        Metric="tracking",
        Reference=Reference,
        ReferenceRates=Rates,
    )
    Metadata.update(
        FlockDiagnostics(
            Motion, Model.Radius, Obstacles=getattr(Model, "Obstacles", None)
        )
    )
    Metadata["SensingRadius"] = np.array([Model.Radius])
    VisualizerInstance = Visualizer("Boids Flocking 3D", Time, Motion, Metadata)
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
