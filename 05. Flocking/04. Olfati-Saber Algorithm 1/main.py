"""Olfati-Saber Algorithm 1."""

from pathlib import Path

import numpy as np
from algorithm import FlockDiagnostics, OlfatiSaberController, Simulate, TriangleLattice
from plotting import Visualizer

# Edit the parameters below, then run this file.
DIM = 2
REFERENCE_FUNCTION = None
SEED = 141
MODEL_PARAMETERS = {
    "Radius": 1.8,
    "Distance": 1.2,
    "Epsilon": 0.1,
    "H": 0.2,
    "A": 5.0,
    "B": 5.0,
    "GradientGain": 2.0,
    "AlignmentGain": 2.0,
    "PositionGain": 0.8,
    "VelocityGain": 2.0,
}
TRIANGLE_LATTICE_PARAMETERS = {"Distance": 1.2}
SIMULATION_PARAMETERS = {"SimTime": 25.0, "Dt": 0.015}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)
    Model = OlfatiSaberController(Reference=REFERENCE_FUNCTION, **MODEL_PARAMETERS)
    Initial = np.zeros((6, 2, DIM))
    Position = TriangleLattice(**TRIANGLE_LATTICE_PARAMETERS)
    if DIM == 3:
        Position = np.column_stack(
            [Position, np.array([-0.4, 0.4, -0.4, 0.4, -0.4, 0.4])]
        )
    Initial[:, 0] = Position + Random.normal(0, 0.03, Position.shape)
    Initial[:, 1] = Random.normal(0, 0.12, Position.shape)
    Initial[:, 1] += [0.25, 0.1]
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Metadata = dict(Kind="archiveflocking", Metric="alignment")
    Metadata.update(
        FlockDiagnostics(
            Motion, Model.Radius, Obstacles=getattr(Model, "Obstacles", None)
        )
    )
    Metadata["SensingRadius"] = np.array([Model.Radius])
    VisualizerInstance = Visualizer("Olfati-Saber Algorithm 1", Time, Motion, Metadata)
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
