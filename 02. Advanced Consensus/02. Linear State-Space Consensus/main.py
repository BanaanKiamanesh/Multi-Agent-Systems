"""Linear State-Space Consensus."""

from pathlib import Path

import numpy as np
from algorithm import (
    ConnectedGraph,
    LinearStateConsensus,
    NormalizedLaplacian,
    Simulate,
)
from plotting import Visualizer

# Edit the parameters below, then run this file.
AGENT_NUM = 6
SEED = 26
MODEL_PARAMETERS = {"PositionGain": 1.8, "VelocityGain": 2.8, "NeighborGain": 0.35}
SIMULATION_PARAMETERS = {"SimTime": 12.0, "Dt": 0.03}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Graph = ConnectedGraph(AgentNum=AGENT_NUM)
    Laplacian, _ = NormalizedLaplacian(Graph)
    Metadata = {}
    Model = LinearStateConsensus(Laplacian, **MODEL_PARAMETERS)
    Initial = np.column_stack(
        [Random.uniform(-3, 3, AGENT_NUM), Random.normal(0, 0.28, AGENT_NUM)]
    )
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(Kind="linear", Laplacian=Laplacian)
    VisualizerInstance = Visualizer(
        "Linear State-Space Consensus", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
