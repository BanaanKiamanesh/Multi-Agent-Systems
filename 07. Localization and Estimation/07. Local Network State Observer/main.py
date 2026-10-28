"""Local Network State Observer."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import FormationOffsets, LocalStateObserver, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
DRIFT = np.array([0.1, 0.04])
SEED = 71
FORMATION_OFFSETS_PARAMETERS = {"AgentNum": 5, "Radius": 1.2}
MODEL_PARAMETERS = {"Gain": 5.0}
SIMULATION_PARAMETERS = {"SimTime": 40.0, "Dt": 0.02}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = nx.cycle_graph(5)
    Offset = FormationOffsets(**FORMATION_OFFSETS_PARAMETERS)

    def Signal(Value):
        return Offset + Value * DRIFT

    Model = LocalStateObserver(
        Graph, Signal, Drift=lambda Value: DRIFT, **MODEL_PARAMETERS
    )
    Initial = Random.normal(0, 1.5, (5, 5, 2))
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Desired = np.stack([Signal(Value) for Value in Time])
    Metadata.update(Kind="frameobserver", DesiredTrajectory=Desired)
    VisualizerInstance = Visualizer(
        "Local Network State Observer", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
