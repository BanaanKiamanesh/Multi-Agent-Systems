"""Finite-Time Consensus."""

from pathlib import Path

import numpy as np
from algorithm import ConnectedGraph, FiniteTimeConsensus, Simulate, WeightedLaplacian
from plotting import Visualizer

# Edit the parameters below, then run this file.
AGENT_NUM = 6
SEED = 47
MODEL_PARAMETERS = {"Gain": 1.5, "Alpha": 0.58}
SIMULATION_PARAMETERS = {"SimTime": 9.0, "Dt": 0.008}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Graph = ConnectedGraph(AgentNum=AGENT_NUM)
    Laplacian, _ = WeightedLaplacian(Graph)
    Metadata = {}
    Model = FiniteTimeConsensus(Graph, **MODEL_PARAMETERS)
    Initial = Random.uniform(-3.0, 3.0, AGENT_NUM)
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(
        Kind="consensus", Laplacian=Laplacian, Alpha=np.array([Model.Alpha])
    )
    VisualizerInstance = Visualizer("Finite-Time Consensus", Time, Motion, Metadata)
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
