"""Heterogeneous Fractional Network."""

from pathlib import Path

import numpy as np
from algorithm import HeterogeneousFractionalNetwork, SimulateFractional
from plotting import Visualizer

# Edit the parameters below, then run this file.
ORDERS = np.array([0.84, 0.81, 0.89, 0.83, 0.92, 0.88, 0.86])
SEED = 42
MODEL_PARAMETERS = {"Gain": 1.1}
SIMULATION_PARAMETERS = {"SimTime": 15.0, "Dt": 0.048}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Model = HeterogeneousFractionalNetwork(**MODEL_PARAMETERS)
    Initial = Random.uniform(-2.0, 2.0, 7)
    Time, Motion = SimulateFractional(
        Model.SysDyn, Initial, Order=ORDERS, **SIMULATION_PARAMETERS
    )
    Metadata.update(Kind="consensus", Orders=ORDERS)
    VisualizerInstance = Visualizer(
        "Heterogeneous Fractional Network", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
