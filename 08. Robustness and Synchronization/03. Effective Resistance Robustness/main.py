"""Effective Resistance Robustness."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import (
    EffectiveResistance,
    FirstOrderConsensus,
    PerturbedLaplacian,
    Simulate,
)
from plotting import Visualizer

# Edit the parameters below, then run this file.
AGENT_NUM = 6
SEED = 56
MODEL_PARAMETERS = {"Gain": 1.0}
SIMULATION_PARAMETERS = {"SimTime": 40.0, "Dt": 0.025}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Initial = Random.uniform(-3.0, 3.0, AGENT_NUM)
    Metadata = {}
    Graph = nx.cycle_graph(6)
    Resistance = EffectiveResistance(Graph, 0, 3)
    Perturbation = -0.65 / Resistance
    Laplacian = PerturbedLaplacian(Graph, 0, 3, Perturbation)
    Model = FirstOrderConsensus(Laplacian, **MODEL_PARAMETERS)
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(
        Kind="consensus",
        Laplacian=Laplacian,
        Resistance=np.array([Resistance]),
        Perturbation=np.array([Perturbation]),
    )
    VisualizerInstance = Visualizer(
        "Effective Resistance Robustness", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
