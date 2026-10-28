"""Olfati-Saber Algorithm 3 3D."""

from pathlib import Path

import numpy as np
from algorithm import (
    FlockDiagnostics,
    ObstacleFlockingController,
    SimulateBoundedFlock,
    TriangleLattice,
)
from plotting import Visualizer

# Edit the parameters below, then run this file.
DIM = 3
MODE = "velocity"
GOAL = np.array([10.0, 4.0, 2.4])
OBSTACLES = np.array([[5, 4.7, 2.4, 0.65], [7.5, 7.5, 2.4, 0.55]])
TRIANGLE_LATTICE_PARAMETERS = {"Distance": 1.2}
MODEL_PARAMETERS = {
    "Radius": 2.0,
    "Distance": 1.1,
    "ObstacleRadius": 4.0,
    "H": 0.2,
    "GradientGain": 0.6,
    "AlignmentGain": 1.0,
    "ObstacleGain": 1.5,
    "GoalGain": 1.2,
    "Damping": 1.0,
    "SafetyBuffer": 0.3,
    "TangentGain": 0.02,
}
SIMULATION_PARAMETERS = {
    "SimTime": 80.0,
    "Dt": 0.015,
    "SpeedLimit": 1.5,
    "ArenaSize": 12.0,
    "GoalRadius": 1.5,
    "Majority": 0.8,
    "Method": "euler",
}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Position = 0.55 * TriangleLattice(**TRIANGLE_LATTICE_PARAMETERS) + [1.4, 4.0]
    Position = np.column_stack([Position, 2.4 + np.linspace(-0.3, 0.3, len(Position))])
    Model = ObstacleFlockingController(
        GOAL, OBSTACLES, TangentMode=MODE, **MODEL_PARAMETERS
    )
    Initial = np.zeros((len(Position), 2, DIM))
    Initial[:, 0] = Position
    Time, Motion = SimulateBoundedFlock(Model, Initial, **SIMULATION_PARAMETERS)
    Fraction = np.mean(
        np.linalg.norm(Motion[:, :, 0] - Model.Goal, axis=2)
        < SIMULATION_PARAMETERS["GoalRadius"],
        axis=1,
    )
    Metadata = dict(
        Kind="archiveflocking",
        Metric="goal",
        Goal=Model.Goal,
        Obstacles=Model.Obstacles,
        GoalFraction=Fraction,
        SpeedLimit=np.array([SIMULATION_PARAMETERS["SpeedLimit"]]),
        ArenaSize=np.array([SIMULATION_PARAMETERS["ArenaSize"]]),
        GoalRadius=np.array([SIMULATION_PARAMETERS["GoalRadius"]]),
        RequiredFraction=np.array([SIMULATION_PARAMETERS["Majority"]]),
    )
    Metadata.update(
        FlockDiagnostics(
            Motion, Model.Radius, Obstacles=getattr(Model, "Obstacles", None)
        )
    )
    Metadata["SensingRadius"] = np.array([Model.Radius])
    VisualizerInstance = Visualizer(
        "Olfati-Saber Algorithm 3 3D", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
