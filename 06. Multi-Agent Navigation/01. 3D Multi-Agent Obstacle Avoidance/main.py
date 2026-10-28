"""3D Multi-Agent Obstacle Avoidance."""

from pathlib import Path

import numpy as np
from algorithm import ObstacleAvoidance3D, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
START = np.array(
    [[-3.0, -1.45, 0.15], [-3.0, 1.45, 0.75], [-2.1, -0.55, 2.4], [-2.1, 0.85, 2.6]]
)
GOALS = np.array(
    [[3.0, -1.45, 0.25], [3.0, 1.45, 0.95], [2.6, -0.55, 2.5], [2.6, 0.85, 2.7]]
)
CENTERS = np.array([[0.0, 0.0, 1.35]])
RADII = np.array([0.95])
MODEL_PARAMETERS = {
    "GoalGain": 1.1,
    "Damping": 2.0,
    "RepulsionGain": 2.0,
    "AgentRadius": 0.16,
    "SafeDistance": 1.15,
}
SIMULATION_PARAMETERS = {"SimTime": 15.0, "Dt": 0.025}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Metadata = {}
    Model = ObstacleAvoidance3D(GOALS, CENTERS, RADII, **MODEL_PARAMETERS)
    Initial = np.zeros((len(START), 2, 3))
    Initial[:, 0] = START
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata.update(Kind="obstacle3d", Goals=GOALS, Centers=CENTERS, Radii=RADII)
    VisualizerInstance = Visualizer(
        "3D Multi-Agent Obstacle Avoidance", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
