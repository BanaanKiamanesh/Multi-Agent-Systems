"""Max-Min Message Consensus."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import MaxMinConsensus
from plotting import Visualizer

# Edit the parameters below, then run this file.
SEED = 88
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Initial = Random.uniform(-3, 3, 7)
    Time, Motion = MaxMinConsensus(nx.path_graph(7), Initial)
    Desired = np.tile([Initial.max(), Initial.min()], (7, 1))
    Metadata.update(
        Kind="targetspatial",
        DesiredPositions=Desired,
        TimeLabel="Message round",
        AxisLabels=("Maximum estimate", "Minimum estimate"),
    )
    VisualizerInstance = Visualizer("Max-Min Message Consensus", Time, Motion, Metadata)
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
