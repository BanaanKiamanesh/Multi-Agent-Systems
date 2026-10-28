"""Swarming and Collective Motion."""

from pathlib import Path

import numpy as np
from algorithm import Simulate, SwarmController, nxCompleteGraph
from plotting import Visualizer

# Edit the parameters below, then run this file.
AGENT_NUM = 6
SEED = 30
MODEL_PARAMETERS = {"Repulsion": 3.5, "Attraction": 0.85, "Damping": 1.8}
SIMULATION_PARAMETERS = {"SimTime": 16.0, "Dt": 0.035}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Model = SwarmController(nxCompleteGraph(AGENT_NUM), **MODEL_PARAMETERS)
    Initial = np.zeros((AGENT_NUM, 2, 2))
    Initial[:, 0] = Random.normal(0, 2.4, (AGENT_NUM, 2))
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(Kind="swarm")
    VisualizerInstance = Visualizer(
        "Swarming and Collective Motion", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
