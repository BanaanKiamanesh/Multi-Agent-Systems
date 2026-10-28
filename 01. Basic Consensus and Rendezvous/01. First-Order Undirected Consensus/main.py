"""First-Order Undirected Consensus."""

from pathlib import Path

import numpy as np
from algorithm import ConnectedGraph, FirstOrderConsensus, Simulate, WeightedLaplacian
from plotting import Visualizer

# Edit the parameters below, then run this file.
AGENT_NUM = 6
SEED = 22
MODEL_PARAMETERS = {"Gain": 1.1}
SIMULATION_PARAMETERS = {"SimTime": 8.0, "Dt": 0.025}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Graph = ConnectedGraph(AgentNum=AGENT_NUM)
    Laplacian, _ = WeightedLaplacian(Graph)
    Initial = Random.uniform(-3.0, 3.0, AGENT_NUM)
    Metadata = {}
    Model = FirstOrderConsensus(Laplacian, **MODEL_PARAMETERS)
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(Kind="consensus", Laplacian=Laplacian)
    VisualizerInstance = Visualizer(
        "First-Order Undirected Consensus", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
