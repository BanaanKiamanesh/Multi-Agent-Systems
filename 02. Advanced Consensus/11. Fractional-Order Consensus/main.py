"""Fractional-Order Consensus."""

from pathlib import Path

import numpy as np
from algorithm import (
    ConnectedGraph,
    NonlinearFractionalConsensus,
    NormalizedLaplacian,
    SimulateFractional,
)
from plotting import Visualizer

# Edit the parameters below, then run this file.
AGENT_NUM = 6
SEED = 29
MODEL_PARAMETERS = {"Gain": 1.0}
SIMULATION_PARAMETERS = {"Order": 0.82, "SimTime": 20, "Dt": 0.045}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Graph = ConnectedGraph(AgentNum=AGENT_NUM)
    Laplacian, _ = NormalizedLaplacian(Graph)
    Initial = Random.uniform(-3.0, 3.0, AGENT_NUM)
    Metadata = {}
    Model = NonlinearFractionalConsensus(Laplacian, **MODEL_PARAMETERS)
    Time, Motion = SimulateFractional(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(Kind="consensus", Laplacian=Laplacian)
    VisualizerInstance = Visualizer(
        "Fractional-Order Consensus", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
