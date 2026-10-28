"""Fourth-Order Consensus."""

from pathlib import Path

import numpy as np
from algorithm import ConnectedGraph, HighOrderConsensus, NormalizedLaplacian, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
AGENT_NUM = 6
SEED = 25
MODEL_PARAMETERS = {"Order": 4, "Gains": [1.0, 4.0, 6.0, 4.0]}
SIMULATION_PARAMETERS = {"SimTime": 22.0, "Dt": 0.035}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Graph = ConnectedGraph(AgentNum=AGENT_NUM)
    Laplacian, _ = NormalizedLaplacian(Graph)
    Metadata = {}
    Model = HighOrderConsensus(Laplacian, **MODEL_PARAMETERS)
    Initial = np.zeros((AGENT_NUM, 4, 1))
    Initial[:, 0, 0] = Random.uniform(-3, 3, AGENT_NUM)
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(Kind="higher", Laplacian=Laplacian)
    VisualizerInstance = Visualizer("Fourth-Order Consensus", Time, Motion, Metadata)
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
