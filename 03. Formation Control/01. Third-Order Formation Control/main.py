"""Third-Order Formation Control."""

from pathlib import Path

import numpy as np
from algorithm import (
    ConnectedGraph,
    FormationController,
    FormationOffsets,
    NormalizedLaplacian,
    Simulate,
)
from plotting import Visualizer

# Edit the parameters below, then run this file.
AGENT_NUM = 6
SEED = 31
FORMATION_OFFSETS_PARAMETERS = {"Radius": 1.45}
MODEL_PARAMETERS = {"PositionGain": 2.0, "VelocityGain": 4.2, "AccelerationGain": 3.5}
SIMULATION_PARAMETERS = {"SimTime": 18.0, "Dt": 0.03}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Graph = ConnectedGraph(AgentNum=AGENT_NUM)
    Metadata = {}
    Offset = FormationOffsets(AgentNum=AGENT_NUM, **FORMATION_OFFSETS_PARAMETERS)
    Model = FormationController(
        NormalizedLaplacian(Graph)[0], Offset, **MODEL_PARAMETERS
    )
    Initial = np.zeros((AGENT_NUM, 3, 2))
    Initial[:, 0] = Random.normal(0, 2.4, (AGENT_NUM, 2)) + np.array([1.5, 0.3])
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(Kind="formation", Offsets=Offset)
    VisualizerInstance = Visualizer(
        "Third-Order Formation Control", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
