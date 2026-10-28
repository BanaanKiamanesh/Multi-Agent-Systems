"""Discrete-Time Consensus."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import SimulateDiscreteConsensus
from plotting import Visualizer

# Edit the parameters below, then run this file.
AGENT_NUM = 6
SEED = 54
SIMULATION_PARAMETERS = {"StepSize": 0.3, "StepNum": 100}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Initial = Random.uniform(-3.0, 3.0, AGENT_NUM)
    Metadata = {}
    Graph = nx.cycle_graph(AGENT_NUM)
    Time, Motion, Mixing = SimulateDiscreteConsensus(
        Graph, Initial, **SIMULATION_PARAMETERS
    )
    Metadata.update(Kind="consensus", Mixing=Mixing, TimeLabel="Iteration")
    VisualizerInstance = Visualizer("Discrete-Time Consensus", Time, Motion, Metadata)
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
