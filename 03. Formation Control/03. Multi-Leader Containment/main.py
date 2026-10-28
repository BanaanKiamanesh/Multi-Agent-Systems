"""Multi-Leader Containment."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import MultiLeaderContainment, Simulate
from plotting import Visualizer
from scipy.spatial import ConvexHull

# Edit the parameters below, then run this file.
INITIAL = np.array(
    [
        [1.0, 1.0],
        [7.0, 1.0],
        [4.0, 6.0],
        [-3.0, -2.0],
        [8.0, -3.0],
        [10.0, 7.0],
        [-1.0, 9.0],
        [8.0, 8.0],
    ]
)
MODEL_PARAMETERS = {"LeaderIndices": (0, 1, 2), "Gain": 1.5}
SIMULATION_PARAMETERS = {"SimTime": 11.0, "Dt": 0.025}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Metadata = {}
    Graph = nx.DiGraph()
    Graph.add_nodes_from(range(8))
    Graph.add_edges_from(
        ((Leader, Follower) for Leader in range(3) for Follower in range(3, 8))
    )
    Graph.add_edges_from(
        (
            ((Index - 3 + Shift) % 5 + 3, Index)
            for Index in range(3, 8)
            for Shift in (1, 2)
        )
    )
    nx.set_edge_attributes(Graph, 1.0, "weight")
    Model = MultiLeaderContainment(Graph, **MODEL_PARAMETERS)
    Time, Motion = Simulate(Model.SysDyn, INITIAL, **SIMULATION_PARAMETERS)
    HullEquation = ConvexHull(INITIAL[:3]).equations
    Metadata.update(
        Kind="containment",
        HullEquation=HullEquation,
        LeaderIndices=Model.LeaderIndices,
        FollowerIndices=Model.FollowerIndices,
    )
    VisualizerInstance = Visualizer("Multi-Leader Containment", Time, Motion, Metadata)
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
