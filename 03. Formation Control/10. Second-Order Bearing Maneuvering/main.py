"""Second-Order Bearing Maneuvering."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import BearingFormation, SecondOrderBearingManeuver, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
DESIRED = np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]])
LEADERS = np.array([0, 1])
DRIFT = np.array([0.15, -0.05])
SCALE_RATE = 0.03
SEED = 84
MODEL_PARAMETERS = {"Gain": 2.0, "Gradient": False}
MODEL_PARAMETERS_2 = {"PositionGain": 2.0, "VelocityGain": 3.0}
SIMULATION_PARAMETERS = {"SimTime": 45.0, "Dt": 0.02}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = nx.Graph([(0, 1), (1, 2), (2, 3), (3, 0), (0, 2)])
    Geometry = BearingFormation(Graph, DESIRED, **MODEL_PARAMETERS)
    Model = SecondOrderBearingManeuver(
        Graph, Geometry.Bearings, LEADERS, DESIRED[LEADERS], **MODEL_PARAMETERS_2
    )
    Initial = np.zeros((4, 2, 2))
    Initial[:, 0] = DESIRED + Random.normal(0, 0.2, DESIRED.shape)
    Initial[LEADERS, 0] = DESIRED[LEADERS]
    Initial[LEADERS, 1] = SCALE_RATE * DESIRED[LEADERS] + DRIFT
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Targets = (1 + SCALE_RATE * Time[:, None, None]) * DESIRED + Time[
        :, None, None
    ] * DRIFT
    Metadata.update(
        Kind="secondbearing",
        DesiredTrajectory=Targets,
        Edges=Geometry.Edges,
        LeaderIndices=LEADERS,
        TargetVelocity=SCALE_RATE * DESIRED + DRIFT,
    )
    VisualizerInstance = Visualizer(
        "Second-Order Bearing Maneuvering", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
