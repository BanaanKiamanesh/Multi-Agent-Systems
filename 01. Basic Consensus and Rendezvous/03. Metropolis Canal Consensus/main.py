"""Metropolis Canal Consensus."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import MetropolisWeights
from plotting import Visualizer

# Edit the parameters below, then run this file.
INITIAL = np.array([1.8, 0.3, 2.4, 0.7, 1.1])
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Metadata = {}
    Canals = nx.Graph([(0, 1), (1, 2), (2, 3), (1, 4), (4, 5)])
    Graph = nx.convert_node_labels_to_integers(nx.line_graph(Canals), ordering="sorted")
    Mixing = MetropolisWeights(Graph)
    Motion = [INITIAL]
    for _ in range(120):
        Motion.append(Mixing @ Motion[-1])
    Motion = np.array(Motion)
    Time = np.arange(len(Motion), dtype=float)
    Metadata.update(
        Kind="consensus",
        Mixing=Mixing,
        CanalEdges=np.array(sorted(Canals.edges())),
        TimeLabel="Iteration",
    )
    VisualizerInstance = Visualizer(
        "Metropolis Canal Consensus", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
