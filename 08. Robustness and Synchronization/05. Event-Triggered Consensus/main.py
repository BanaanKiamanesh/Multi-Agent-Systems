"""Event-Triggered Consensus."""

from pathlib import Path

import numpy as np
from algorithm import ConnectedGraph, SimulateEventTriggeredConsensus, WeightedLaplacian
from plotting import Visualizer

# Edit the parameters below, then run this file.
AGENT_NUM = 6
SEED = 50
SIMULATION_PARAMETERS = {
    "SimTime": 14.0,
    "Dt": 0.02,
    "Gain": 1.5,
    "Threshold": 0.045,
    "Decay": 0.32,
    "MinimumThreshold": 0.0006,
}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Graph = ConnectedGraph(AgentNum=AGENT_NUM)
    Laplacian, _ = WeightedLaplacian(Graph)
    Metadata = {}
    Initial = Random.uniform(-3.0, 3.0, AGENT_NUM)
    Time, Motion, Events = SimulateEventTriggeredConsensus(
        Graph, Initial, **SIMULATION_PARAMETERS
    )
    Metadata.update(
        Kind="consensus",
        Laplacian=Laplacian,
        Events=Events,
        EventCount=np.sum(Events, axis=0),
    )
    VisualizerInstance = Visualizer("Event-Triggered Consensus", Time, Motion, Metadata)
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
