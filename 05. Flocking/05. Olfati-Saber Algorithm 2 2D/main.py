"""Olfati-Saber Algorithm 2 2D."""

from pathlib import Path

import numpy as np
from algorithm import FlockDiagnostics, OlfatiSaberController, Simulate, TriangleLattice
from plotting import Visualizer

# Edit the parameters below, then run this file.
DIM = 2
SEED = 142
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
SIMULATION_PARAMETERS = {"SimTime": 60.0, "Dt": 0.015}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"
SAVE_ANIMATION = False


def CircleReference(TimeValue, Dim=2):
    Angle, Rate = (0.08 * TimeValue, 0.08)
    Position = np.array([1.5 * np.cos(Angle), 1.5 * np.sin(Angle)])
    Velocity = np.array([-1.5 * Rate * np.sin(Angle), 1.5 * Rate * np.cos(Angle)])
    if Dim == 3:
        Position = np.append(Position, 0.03 * TimeValue)
        Velocity = np.append(Velocity, 0.03)
    return (Position, Velocity)


def main():
    # Graph, reference geometry and initial conditions.
    Random = np.random.default_rng(SEED)

    def ReferenceFunction(Value):
        return CircleReference(Value, Dim=DIM)

    Model = OlfatiSaberController(Reference=ReferenceFunction, **MODEL_PARAMETERS)
    Initial = np.zeros((6, 2, DIM))
    Position = TriangleLattice(**TRIANGLE_LATTICE_PARAMETERS)
    if DIM == 3:
        Position = np.column_stack(
            [Position, np.array([-0.4, 0.4, -0.4, 0.4, -0.4, 0.4])]
        )
    Initial[:, 0] = Position + Random.normal(0, 0.03, Position.shape)
    Initial[:, 1] = Random.normal(0, 0.12, Position.shape)
    Initial[:, 0] += ReferenceFunction(0.0)[0]
    Initial[:, 1] += ReferenceFunction(0.0)[1]
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Reference = np.array([ReferenceFunction(Value)[0] for Value in Time])
    Rates = np.array([ReferenceFunction(Value)[1] for Value in Time])
    Metadata = dict(
        Kind="archiveflocking",
        Metric="tracking",
        Reference=Reference,
        ReferenceRates=Rates,
    )
    Metadata.update(
        FlockDiagnostics(
            Motion, Model.Radius, Obstacles=getattr(Model, "Obstacles", None)
        )
    )
    Metadata["SensingRadius"] = np.array([Model.Radius])
    VisualizerInstance = Visualizer(
        "Olfati-Saber Algorithm 2 2D", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)
    if SAVE_ANIMATION:
        VisualizerInstance.SaveAnimation(OUTPUT_DIRECTORY / "agents_animation.gif")


if __name__ == "__main__":
    main()
