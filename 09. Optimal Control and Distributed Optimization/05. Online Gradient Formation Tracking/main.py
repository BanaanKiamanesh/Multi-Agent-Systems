"""Online Gradient Formation Tracking."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import OnlineFormationController, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
DESIRED = np.array([[-0.8, -0.8], [0.8, -0.8], [0.8, 0.8], [-0.8, 0.8]])
SEED = 72
MODEL_PARAMETERS = {
    "ObserverGain": 8.0,
    "ReferenceVelocity": (0.25, -0.1),
    "CentroidPositionGain": 1.5,
    "CentroidVelocityGain": 2.5,
    "FormationGain": 0.5,
    "FormationRateGain": 0.4,
    "AlignmentGain": 0.6,
}
SIMULATION_PARAMETERS = {"SimTime": 25.0, "Dt": 0.01}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = nx.Graph([(0, 1), (1, 2), (2, 3), (3, 0), (0, 2)])
    Model = OnlineFormationController(Graph, DESIRED, **MODEL_PARAMETERS)
    Initial = np.zeros((4, 2, 2))
    Initial[:, 0] = DESIRED + [-0.5, 0.3] + Random.normal(0, 0.1, DESIRED.shape)
    Time, State = Simulate(
        Model.SysDyn, Model.InitialState(Initial), **SIMULATION_PARAMETERS
    )
    Motion = State[:, :16].reshape(-1, 4, 2, 2)
    PositionBank = State[:, 16:48].reshape(-1, 4, 4, 2)
    Metadata.update(
        Kind="optimalformation",
        Edges=Model.Edges,
        Distances=np.sqrt(Model.SquaredDistances),
        Reference=Time[:, None] * Model.ReferenceVelocity,
        ReferenceVelocity=Model.ReferenceVelocity,
        PositionBank=PositionBank,
    )
    VisualizerInstance = Visualizer(
        "Online Gradient Formation Tracking", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
