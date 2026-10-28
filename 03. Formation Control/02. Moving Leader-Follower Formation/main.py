"""Moving Leader-Follower Formation."""

from pathlib import Path

import numpy as np
from algorithm import (
    ConnectedGraph,
    FormationOffsets,
    LeaderFollowerFormation,
    Simulate,
)
from plotting import Visualizer

# Edit the parameters below, then run this file.
AGENT_NUM = 6
SEED = 43
FORMATION_OFFSETS_PARAMETERS = {"Radius": 1.25}
MODEL_PARAMETERS = {
    "Gain": 2.4,
    "Damping": 3.0,
    "Pinning": 1.5,
    "ReferenceSpeed": 0.5,
    "ReferenceAmplitude": 1.0,
    "ReferenceFrequency": 0.25,
    "LeaderPositionGain": 6.0,
    "LeaderVelocityGain": 4.0,
}
SIMULATION_PARAMETERS = {"SimTime": 13.0, "Dt": 0.03}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Metadata = {}
    Graph = ConnectedGraph(AgentNum=AGENT_NUM)
    Offsets = FormationOffsets(AgentNum=AGENT_NUM, **FORMATION_OFFSETS_PARAMETERS)
    Model = LeaderFollowerFormation(Graph, Offsets, **MODEL_PARAMETERS)
    Initial = np.zeros((AGENT_NUM, 2, 2))
    Initial[:, 0] = Random.uniform(-2, 2, (AGENT_NUM, 2)) + Offsets
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Reference = np.stack([Model.Reference(Value) for Value in Time])
    Metadata.update(Kind="movingformation", Offsets=Offsets, Reference=Reference)
    VisualizerInstance = Visualizer(
        "Moving Leader-Follower Formation", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
