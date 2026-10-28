"""Signed Nonlinear Consensus."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import SignedNonlinearConsensus, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
SEED = 79
MODEL_PARAMETERS = {"Gain": 1.3}
SIMULATION_PARAMETERS = {"SimTime": 35.0, "Dt": 0.025}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = nx.cycle_graph(6)
    nx.set_edge_attributes(Graph, 1.0, "weight")
    Graph.add_edge(0, 3, weight=-0.15)
    Model = SignedNonlinearConsensus(Graph, **MODEL_PARAMETERS)
    Initial = Random.uniform(-0.4, 0.4, 6)
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Potential = np.array([Model.Potential(State) for State in Motion])
    Metadata.update(Kind="consensus", Potential=Potential, SignedWeights=Model.Weights)
    VisualizerInstance = Visualizer(
        "Signed Nonlinear Consensus", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
