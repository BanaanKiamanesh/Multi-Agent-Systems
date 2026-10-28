"""Centroidal Voronoi Coverage."""

from pathlib import Path

import numpy as np
from algorithm import LloydCoverage, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
SEED = 33
MODEL_PARAMETERS = {
    "Bounds": ((0.0, 10.0), (0.0, 10.0)),
    "GridSize": 41,
    "SensingRadius": np.inf,
    "Gain": 1.2,
}
SIMULATION_PARAMETERS = {"SimTime": 9.0, "Dt": 0.06}
COVERAGE_COST_PARAMETERS = {"TimeValue": 0.0}
COVERAGE_COST_PARAMETERS_2 = {"TimeValue": 0.0}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Model = LloydCoverage(**MODEL_PARAMETERS)
    Initial = Random.uniform(1, 9, (7, 2))
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(
        Kind="coverage",
        Samples=Model.Samples,
        Density=Model.Density,
        InitialCost=Model.CoverageCost(Initial, **COVERAGE_COST_PARAMETERS),
        FinalCost=Model.CoverageCost(Motion[-1], **COVERAGE_COST_PARAMETERS_2),
    )
    VisualizerInstance = Visualizer(
        "Centroidal Voronoi Coverage", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
