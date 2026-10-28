"""First-Order Directed Consensus."""

from pathlib import Path

import numpy as np
from algorithm import FirstOrderConsensus, RingGraph, Simulate, WeightedLaplacian
from plotting import Visualizer

# Edit the parameters below, then run this file.
SEED = 23
RING_GRAPH_PARAMETERS = {
    "AgentNum": 7,
    "Directed": True,
    "Weights": [1.2, 0.9, 1.4, 1.0, 1.2, 0.8, 1.0],
}
MODEL_PARAMETERS = {"Gain": 1.5}
SIMULATION_PARAMETERS = {"SimTime": 13.0, "Dt": 0.03}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = RingGraph(**RING_GRAPH_PARAMETERS)
    Laplacian, _ = WeightedLaplacian(Graph)
    Initial = Random.uniform(-2.8, 2.8, len(Graph))
    Model = FirstOrderConsensus(Laplacian, **MODEL_PARAMETERS)
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(Kind="consensus", Laplacian=Laplacian)
    VisualizerInstance = Visualizer(
        "First-Order Directed Consensus", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
