"""Proximal Relative Estimation."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import FormationOffsets, SimulateProximalEstimation
from plotting import Visualizer

# Edit the parameters below, then run this file.
SEED = 67
FORMATION_OFFSETS_PARAMETERS = {"AgentNum": 5, "Radius": 1.25}
SIMULATION_PARAMETERS = {"Rho": 2.0, "StepNum": 240}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = nx.wheel_graph(6)
    Desired = np.vstack([np.zeros(2), FormationOffsets(**FORMATION_OFFSETS_PARAMETERS)])
    Desired -= Desired[0]
    Edges = np.array(list(Graph.edges()), dtype=int)
    Measurements = Desired[Edges[:, 1]] - Desired[Edges[:, 0]]
    Initial = Random.normal(0, 2.0, Desired.shape)
    Time, Motion = SimulateProximalEstimation(
        Graph, Measurements, Initial, Desired[0], **SIMULATION_PARAMETERS
    )
    Metadata.update(
        Kind="targetspatial",
        DesiredPositions=Desired,
        Edges=Edges,
        AnchorIndices=np.array([0]),
        Measurements=Measurements,
        TimeLabel="Iteration",
    )
    VisualizerInstance = Visualizer(
        "Proximal Relative Estimation", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
