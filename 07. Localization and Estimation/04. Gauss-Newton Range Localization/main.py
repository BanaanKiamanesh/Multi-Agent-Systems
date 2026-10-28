"""Gauss-Newton Range Localization."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import FormationOffsets, RangeFrameEstimator
from plotting import Visualizer

# Edit the parameters below, then run this file.
ANCHORS = np.array([0, 1, 2])
SEED = 69
FORMATION_OFFSETS_PARAMETERS = {"AgentNum": 5, "Radius": 1.25}
MODEL_PARAMETERS = {"Gain": 0.8}
GAUSS_NEWTON_PARAMETERS = {"StepNum": 30, "Damping": 1e-05, "InnerSteps": 300}
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
    Ranges = np.linalg.norm(Desired[Edges[:, 0]] - Desired[Edges[:, 1]], axis=1)
    Model = RangeFrameEstimator(
        Graph, Ranges, ANCHORS, Desired[ANCHORS], **MODEL_PARAMETERS
    )
    Initial = Desired + Random.normal(0, 0.18, Desired.shape)
    Initial[ANCHORS] = Desired[ANCHORS]
    Time, Motion = Model.GaussNewton(Initial, **GAUSS_NEWTON_PARAMETERS)
    Metadata["TimeLabel"] = "Outer iteration"
    Residuals = np.array(
        [np.linalg.norm(Model.ResidualJacobian(Position)[0]) for Position in Motion]
    )
    Metadata.update(
        Kind="targetspatial",
        DesiredPositions=Desired,
        Edges=Edges,
        Ranges=Ranges,
        AnchorIndices=ANCHORS,
        RangeResidual=Residuals,
    )
    VisualizerInstance = Visualizer(
        "Gauss-Newton Range Localization", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
