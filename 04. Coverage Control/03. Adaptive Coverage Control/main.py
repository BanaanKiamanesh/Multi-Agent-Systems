"""Adaptive Coverage Control."""

from pathlib import Path

import numpy as np
from algorithm import AdaptiveCoverage, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
SEED = 45
MODEL_PARAMETERS = {
    "Bounds": ((0.0, 10.0), (0.0, 10.0)),
    "GridSize": 36,
    "SensingRadius": np.inf,
    "Gain": 1.25,
}
SIMULATION_PARAMETERS = {"SimTime": 14, "Dt": 0.08}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Model = AdaptiveCoverage(**MODEL_PARAMETERS)
    Initial = Random.uniform(0.5, 9.5, (6, 2))
    InitialCost = Model.CoverageCost(Initial)
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(
        Kind="coverage",
        Samples=Model.Samples,
        Density=Model.DensityAt(Time[-1]),
        InitialCost=InitialCost,
        FinalCost=Model.CoverageCost(Motion[-1], Time[-1]),
        DensityAdaptive=True,
    )
    VisualizerInstance = Visualizer("Adaptive Coverage Control", Time, Motion, Metadata)
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
