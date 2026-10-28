"""Second-Order Consensus."""

from pathlib import Path

import numpy as np
from algorithm import ConnectedGraph, HighOrderConsensus, NormalizedLaplacian, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
AGENT_NUM = 6
SEED = 24
MODEL_PARAMETERS = {"Order": 2, "Gains": [2.0, 2.5]}
SIMULATION_PARAMETERS = {"SimTime": 12.0, "Dt": 0.03}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Graph = ConnectedGraph(AgentNum=AGENT_NUM)
    Laplacian, _ = NormalizedLaplacian(Graph)
    Metadata = {}
    Model = HighOrderConsensus(Laplacian, **MODEL_PARAMETERS)
    Initial = np.zeros((AGENT_NUM, 2, 1))
    Initial[:, 0, 0] = Random.uniform(-3, 3, AGENT_NUM)
    Initial[:, 1, 0] = Random.normal(0, 0.2, AGENT_NUM)
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(Kind="higher", Laplacian=Laplacian)
    VisualizerInstance = Visualizer("Second-Order Consensus", Time, Motion, Metadata)
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
