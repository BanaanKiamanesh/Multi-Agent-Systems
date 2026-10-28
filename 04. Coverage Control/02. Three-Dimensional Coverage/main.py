"""Three-Dimensional Coverage."""

from pathlib import Path

import numpy as np
from algorithm import LloydCoverage3D, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
SEED = 44
MODEL_PARAMETERS = {"Bounds": ((0, 8), (0, 8), (0, 6)), "GridSize": 15, "Gain": 1.0}
SIMULATION_PARAMETERS = {"SimTime": 9.0, "Dt": 0.095}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Model = LloydCoverage3D(**MODEL_PARAMETERS)
    Initial = Random.uniform([0.5, 0.5, 0.5], [7.5, 7.5, 5.5], (6, 3))
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(
        Kind="coverage3d",
        Samples=Model.Samples,
        Density=Model.Density,
        InitialCost=Model.CoverageCost(Initial),
        FinalCost=Model.CoverageCost(Motion[-1]),
    )
    VisualizerInstance = Visualizer(
        "Three-Dimensional Coverage", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
