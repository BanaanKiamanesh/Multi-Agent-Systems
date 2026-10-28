"""Connectivity-Preserving Formation."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import ConnectivityPreservingFormation, FormationOffsets, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
FORMATION_OFFSETS_PARAMETERS = {"AgentNum": 5, "Radius": 0.8}
MODEL_PARAMETERS = {"CommunicationRange": 2.5, "Gain": 0.6, "BarrierGain": 0.15}
SIMULATION_PARAMETERS = {"SimTime": 18.0, "Dt": 0.015}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Metadata = {}
    Graph = nx.cycle_graph(5)
    Initial = FormationOffsets(**FORMATION_OFFSETS_PARAMETERS)
    Desired = 3.2 * Initial + np.array([1.0, 0.5])
    Model = ConnectivityPreservingFormation(Graph, Desired, **MODEL_PARAMETERS)
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Lengths = np.linalg.norm(
        Motion[:, Model.Edges[:, 0]] - Motion[:, Model.Edges[:, 1]], axis=2
    )
    Metadata.update(
        Kind="targetspatial",
        DesiredPositions=Desired,
        Edges=Model.Edges,
        EdgeLengths=Lengths,
        CommunicationRange=np.array([Model.CommunicationRange]),
    )
    VisualizerInstance = Visualizer(
        "Connectivity-Preserving Formation", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
