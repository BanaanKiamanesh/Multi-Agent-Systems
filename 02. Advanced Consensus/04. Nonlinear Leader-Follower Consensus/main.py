"""Nonlinear Leader-Follower Consensus."""

from pathlib import Path

import numpy as np
from algorithm import NonlinearLeaderFollower, RingGraph, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
AGENT_NUM = 6
SEED = 27
RING_GRAPH_PARAMETERS = {"Directed": False, "Weights": None}
MODEL_PARAMETERS = {
    "Gain": 1.8,
    "PinningGain": 2.5,
    "ReferenceOffset": 1.0,
    "ReferenceAmplitude": 0.65,
    "ReferenceFrequency": 0.32,
}
SIMULATION_PARAMETERS = {"SimTime": 12.0, "Dt": 0.025}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = RingGraph(AgentNum=AGENT_NUM, **RING_GRAPH_PARAMETERS)
    Model = NonlinearLeaderFollower(Graph, **MODEL_PARAMETERS)
    Initial = Random.uniform(-3, 3, AGENT_NUM)
    Initial[0] = Model.Reference(0)
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(Kind="tracking", Reference=Model.Reference(Time))
    VisualizerInstance = Visualizer(
        "Nonlinear Leader-Follower Consensus", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
