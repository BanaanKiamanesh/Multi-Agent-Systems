"""Velocity-Aligned Flocking."""

from pathlib import Path

import numpy as np
from algorithm import FlockingController, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
SEED = 81
MODEL_PARAMETERS = {
    "Distance": 1.2,
    "SensingRange": 1.8,
    "Epsilon": 0.1,
    "PositionGain": 0.08,
    "VelocityGain": 1.2,
    "ReferenceVelocity": (0.4, 0.1),
}
SIMULATION_PARAMETERS = {"SimTime": 45.0, "Dt": 0.015}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Model = FlockingController(**MODEL_PARAMETERS)
    Initial = np.zeros((6, 2, 2))
    Distance = Model.Distance
    Height = np.sqrt(3) * Distance / 2
    Lattice = np.array(
        [
            [0.0, 0.0],
            [Distance, 0.0],
            [2 * Distance, 0.0],
            [Distance / 2, Height],
            [1.5 * Distance, Height],
            [Distance, 2 * Height],
        ]
    )
    Lattice -= Lattice.mean(axis=0)
    Initial[:, 0] = Lattice + Random.normal(0, 0.04, (6, 2))
    Initial[:, 1] = Model.ReferenceVelocity + Random.normal(0, 0.1, (6, 2))
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(
        Kind="flocking",
        ReferenceVelocity=Model.ReferenceVelocity,
        SensingRange=np.array([Model.SensingRange]),
    )
    VisualizerInstance = Visualizer("Velocity-Aligned Flocking", Time, Motion, Metadata)
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
