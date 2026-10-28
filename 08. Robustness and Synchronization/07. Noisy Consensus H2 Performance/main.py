"""Noisy Consensus H2 Performance."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import NoisyConsensus
from plotting import Visualizer

# Edit the parameters below, then run this file.
SIMULATION_PARAMETERS = {"SimTime": 100.0, "Dt": 0.05, "EnsembleNum": 128, "Seed": 92}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Metadata = {}
    Time, Motion, Covariance, Variance, Running = NoisyConsensus(
        nx.cycle_graph(6), **SIMULATION_PARAMETERS
    )
    Metadata.update(
        Kind="noisyconsensus",
        Covariance=Covariance,
        EnsembleVariance=Variance,
        RunningVariance=Running,
        TheoreticalVariance=np.array([np.trace(Covariance)]),
    )
    VisualizerInstance = Visualizer(
        "Noisy Consensus H2 Performance", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
