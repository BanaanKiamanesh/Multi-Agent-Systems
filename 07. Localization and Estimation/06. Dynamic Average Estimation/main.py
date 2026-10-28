"""Dynamic Average Estimation."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import DynamicAverageEstimator, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
SEED = 66
MODEL_PARAMETERS = {"Gamma": 3.0, "ProportionalGain": 3.0, "IntegralGain": 3.0}
SIMULATION_PARAMETERS = {"SimTime": 65.0, "Dt": 0.025}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = nx.cycle_graph(6)
    Offset, Amplitude = Random.uniform(-2, 2, (2, 6))

    def Input(Value):
        return (
            Offset
            + Amplitude * np.sin(0.4 * Value) * np.exp(-0.2 * Value)
            + 0.6 * (1 - np.exp(-0.1 * Value))
        )

    Model = DynamicAverageEstimator(Graph, Input, **MODEL_PARAMETERS)
    Initial = Random.normal(0, 0.6, (6, 2))
    Time, State = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Motion = State[:, :, 0]
    Inputs = np.stack([Input(Value) for Value in Time])
    Metadata.update(
        Kind="tracking",
        Reference=Inputs.mean(axis=1),
        Inputs=Inputs,
        IntegralState=State[:, :, 1],
    )
    VisualizerInstance = Visualizer(
        "Dynamic Average Estimation", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
