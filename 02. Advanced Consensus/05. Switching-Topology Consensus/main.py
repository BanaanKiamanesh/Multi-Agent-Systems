"""Switching-Topology Consensus."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import Simulate, SwitchingTopologyConsensus, WeightedLaplacian
from plotting import Visualizer

# Edit the parameters below, then run this file.
AGENT_NUM = 6
SEED = 46
MODEL_PARAMETERS = {"Gain": 1.35, "SwitchPeriod": 0.8}
SIMULATION_PARAMETERS = {"SimTime": 12.0, "Dt": 0.02}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Topologies = [
        nx.cycle_graph(AGENT_NUM),
        nx.star_graph(AGENT_NUM - 1),
        nx.path_graph(AGENT_NUM),
    ]
    Laplacians = np.stack(
        [WeightedLaplacian(GraphValue)[0] for GraphValue in Topologies]
    )
    Initial = Random.uniform(-3.0, 3.0, AGENT_NUM)
    Model = SwitchingTopologyConsensus(Laplacians, **MODEL_PARAMETERS)
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(
        Kind="consensus",
        Laplacians=Laplacians,
        SwitchPeriod=np.array([Model.SwitchPeriod]),
    )
    VisualizerInstance = Visualizer(
        "Switching-Topology Consensus", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
