"""Shrinking-Horizon Preference Agreement."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import PreferenceAgreement
from plotting import Visualizer

# Edit the parameters below, then run this file.
PREFERENCES = np.array([-1.5, 0.3, 1.5, -0.6, 0.8])
WEIGHTS = np.array([0.7, 1.0, 1.4, 0.8, 1.2])
INPUT_WEIGHTS = np.full(len(PREFERENCES), 2.0)
SEED = 87
SHRINKING_HORIZON_PARAMETERS = {"Horizon": 50, "Sweeps": 3}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = nx.path_graph(len(PREFERENCES))
    Model = PreferenceAgreement(Graph, PREFERENCES, WEIGHTS, INPUT_WEIGHTS)
    Initial = Random.uniform(-2, 2, len(PREFERENCES))
    Motion, Controls, History, Multipliers = Model.ShrinkingHorizon(
        Initial, **SHRINKING_HORIZON_PARAMETERS
    )
    Metadata["PredictedTerminalError"] = History
    Time = np.arange(len(Motion), dtype=float)
    Metadata.update(
        Kind="consensus",
        Controls=Controls,
        Multipliers=Multipliers,
        Preferences=PREFERENCES,
        StateWeights=WEIGHTS,
        InputWeights=INPUT_WEIGHTS,
        Objective=np.array([Model.Cost(Motion, Controls)]),
        TimeLabel="Physical time step",
    )
    VisualizerInstance = Visualizer(
        "Shrinking-Horizon Preference Agreement", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
