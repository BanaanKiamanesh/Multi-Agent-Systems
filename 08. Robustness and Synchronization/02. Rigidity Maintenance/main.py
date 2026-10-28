"""Rigidity Maintenance."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import RigidityEigenpair, RigidityMaintenance, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
INITIAL = np.array([[-1.0, -0.8], [1.0, -0.6], [1.2, 0.9]])
DESIRED = np.array([[-1.5, 0.0], [-0.4, 0.0], [1.5, 0.0]])
MODEL_PARAMETERS = {"Gain": 0.5, "BarrierGain": 0.06, "MinimumEigenvalue": 0.1}
SIMULATION_PARAMETERS = {"SimTime": 12.0, "Dt": 0.01}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Metadata = {}
    Graph = nx.complete_graph(3)
    Model = RigidityMaintenance(Graph, DESIRED, **MODEL_PARAMETERS)
    Time, Motion = Simulate(Model.SysDyn, INITIAL, **SIMULATION_PARAMETERS)
    Eigenvalues = np.array(
        [RigidityEigenpair(Model.Edges, Position)[0] for Position in Motion]
    )
    Metadata.update(
        Kind="targetspatial",
        DesiredPositions=DESIRED,
        Edges=Model.Edges,
        RigidityEigenvalues=Eigenvalues,
        MinimumEigenvalue=np.array([Model.MinimumEigenvalue]),
    )
    VisualizerInstance = Visualizer("Rigidity Maintenance", Time, Motion, Metadata)
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
