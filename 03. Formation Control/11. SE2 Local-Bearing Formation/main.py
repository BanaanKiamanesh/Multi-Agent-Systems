"""SE2 Local-Bearing Formation."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import SE2BearingFormation, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
DESIRED = np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]])
HEADINGS = np.array([0.0, 0.2, -0.3, 0.1])
SEED = 89
MODEL_PARAMETERS = {"PositionGain": 2.0, "HeadingGain": 2.0}
SIMULATION_PARAMETERS = {"SimTime": 50.0, "Dt": 0.02}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = nx.Graph([(0, 1), (1, 2), (2, 3), (3, 0), (0, 2)])
    Model = SE2BearingFormation(Graph, DESIRED, HEADINGS, **MODEL_PARAMETERS)
    Initial = np.column_stack(
        [
            DESIRED + Random.normal(0, 0.12, DESIRED.shape),
            HEADINGS + Random.normal(0, 0.15, 4),
        ]
    )
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Potential = np.array([Model.Potential(State) for State in Motion])
    Metadata.update(
        Kind="se2bearing",
        BearingPotential=Potential,
        LocalTargets=Model.LocalTargets,
        Edges=Model.Edges,
    )
    VisualizerInstance = Visualizer(
        "SE2 Local-Bearing Formation", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
