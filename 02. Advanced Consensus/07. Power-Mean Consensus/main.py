"""Power-Mean Consensus."""

from pathlib import Path

import networkx as nx
import numpy as np
from algorithm import PowerMeanConsensus, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
SEED = 78
MODEL_PARAMETERS = {"Power": 2.0, "Gain": 1.0}
SIMULATION_PARAMETERS = {"SimTime": 18.0, "Dt": 0.01}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = nx.cycle_graph(6)
    Initial = Random.uniform(0.5, 4.0, 6)
    Model = PowerMeanConsensus(Graph, **MODEL_PARAMETERS)
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(
        Kind="consensus",
        PowerMean=np.array(
            [
                (
                    np.exp(np.mean(np.log(Initial)))
                    if MODEL_PARAMETERS["Power"] == 0
                    else np.mean(Initial ** MODEL_PARAMETERS["Power"])
                    ** (1 / MODEL_PARAMETERS["Power"])
                )
            ]
        ),
    )
    VisualizerInstance = Visualizer("Power-Mean Consensus", Time, Motion, Metadata)
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
