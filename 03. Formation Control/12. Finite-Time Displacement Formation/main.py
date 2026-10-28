"""Finite-Time Displacement Formation."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import FiniteTimeFormation, FormationOffsets, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
SEED = 80
FORMATION_OFFSETS_PARAMETERS = {"AgentNum": 5, "Radius": 1.2}
MODEL_PARAMETERS = {"Gain": 1.2, "Alpha": 0.6}
SIMULATION_PARAMETERS = {"SimTime": 8.0, "Dt": 0.003}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = nx.cycle_graph(5)
    Desired = FormationOffsets(**FORMATION_OFFSETS_PARAMETERS)
    Model = FiniteTimeFormation(Graph, Desired, **MODEL_PARAMETERS)
    Initial = Desired + Random.normal(0, 0.7, Desired.shape) + [0.7, -0.2]
    Time, Position = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Motion = Position[:, :, None, :]
    Metadata.update(Kind="formation", Offsets=Desired)
    VisualizerInstance = Visualizer(
        "Finite-Time Displacement Formation", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
