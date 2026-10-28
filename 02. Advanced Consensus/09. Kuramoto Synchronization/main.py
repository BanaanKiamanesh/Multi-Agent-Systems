"""Kuramoto Synchronization."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import KuramotoNetwork, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
AGENT_NUM = 6
SEED = 57
MODEL_PARAMETERS = {"Gain": 2.0}
SIMULATION_PARAMETERS = {"SimTime": 12.0, "Dt": 0.025}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = nx.cycle_graph(AGENT_NUM)
    Frequencies = np.full(AGENT_NUM, 0.7)
    Model = KuramotoNetwork(Graph, Frequencies, **MODEL_PARAMETERS)
    Initial = Random.uniform(-0.9, 0.9, AGENT_NUM)
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Rates = np.stack([Model.SysDyn(Value, Phase) for Value, Phase in zip(Time, Motion)])
    Metadata.update(Kind="oscillator", Frequencies=Frequencies, PhaseRates=Rates)
    VisualizerInstance = Visualizer("Kuramoto Synchronization", Time, Motion, Metadata)
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
