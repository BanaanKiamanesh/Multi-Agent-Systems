"""Planar Rendezvous."""

from pathlib import Path

import numpy as np
from algorithm import FirstOrderConsensus, RingGraph, Simulate, WeightedLaplacian
from plotting import Visualizer

# Edit the parameters below, then run this file.
SEED = 32
RING_GRAPH_PARAMETERS = {"AgentNum": 5, "Directed": True, "Weights": [4, 6, 8, 10, 12]}
MODEL_PARAMETERS = {"Gain": 0.12}
SIMULATION_PARAMETERS = {"SimTime": 13.0, "Dt": 0.03}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = RingGraph(**RING_GRAPH_PARAMETERS)
    Laplacian, _ = WeightedLaplacian(Graph)
    Initial = Random.uniform(-4, 4, (5, 2))
    Model = FirstOrderConsensus(Laplacian, **MODEL_PARAMETERS)
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(Kind="spatial", Laplacian=Laplacian)
    VisualizerInstance = Visualizer("Planar Rendezvous", Time, Motion, Metadata)
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
