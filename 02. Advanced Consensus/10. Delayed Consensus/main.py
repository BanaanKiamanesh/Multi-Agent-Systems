"""Delayed Consensus."""

from pathlib import Path

import numpy as np
from algorithm import ConnectedGraph, NormalizedLaplacian, SimulateDelay
from plotting import Visualizer

# Edit the parameters below, then run this file.
AGENT_NUM = 6
SEED = 28
SIMULATION_PARAMETERS = {"Delay": 0.35, "Gain": 0.95, "SimTime": 18.0, "Dt": 0.02}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Graph = ConnectedGraph(AgentNum=AGENT_NUM)
    Laplacian, _ = NormalizedLaplacian(Graph)
    Initial = Random.uniform(-3.0, 3.0, AGENT_NUM)
    Metadata = {}
    Time, Motion = SimulateDelay(Laplacian, Initial, **SIMULATION_PARAMETERS)
    Step = Time[1] - Time[0]
    Metadata.update(
        Kind="consensus",
        Laplacian=Laplacian,
        EffectiveDelay=np.array(
            [np.rint(SIMULATION_PARAMETERS["Delay"] / Step) * Step]
        ),
        Gain=np.array([SIMULATION_PARAMETERS["Gain"]]),
    )
    VisualizerInstance = Visualizer("Delayed Consensus", Time, Motion, Metadata)
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
