"""Objective-Coded Robust Consensus."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import DecodeWeights, FirstOrderConsensus, MultiEdgeRobustness, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
CODEWORDS = np.full(2, np.arctanh(0.5))
TAMPERING = np.array([-0.12, -0.1])
SEED = 74
DECODE_WEIGHTS_PARAMETERS = {"WeightLimit": 2.0}
MODEL_PARAMETERS = {"Gain": 1.25}
SIMULATION_PARAMETERS = {"SimTime": 25.0, "Dt": 0.025}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = nx.cycle_graph(6)
    Weights = DecodeWeights(CODEWORDS + TAMPERING, **DECODE_WEIGHTS_PARAMETERS)
    Edges = [(0, 1), (3, 4)]
    Laplacian, Certificate = MultiEdgeRobustness(Graph, Edges, 1.0 - Weights)
    Model = FirstOrderConsensus(Laplacian, **MODEL_PARAMETERS)
    Initial = Random.uniform(-3, 3, 6)
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(
        Kind="consensus",
        Laplacian=Laplacian,
        Codewords=CODEWORDS,
        Tampering=TAMPERING,
        DecodedWeights=Weights,
        RobustnessCertificate=Certificate,
    )
    VisualizerInstance = Visualizer(
        "Objective-Coded Robust Consensus", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
