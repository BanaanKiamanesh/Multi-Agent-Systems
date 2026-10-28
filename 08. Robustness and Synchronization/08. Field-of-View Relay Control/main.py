"""Field-of-View Relay Control."""

from pathlib import Path

import numpy as np
from algorithm import FieldOfViewRelay, Simulate
from plotting import Visualizer

# Edit the parameters below, then run this file.
MODEL_PARAMETERS = {"HalfAngle": 0.65, "Gain": 3.0}
SIMULATION_PARAMETERS = {"SimTime": 25.0, "Dt": 0.005}
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results"


def main():
    # Graph, reference geometry and initial conditions.
    Metadata = {}

    def Targets(Value):
        return np.array(
            [
                [2.0 + 0.1 * Value, 0.8 + 0.15 * Value],
                [2.0 + 0.1 * Value, -0.8 - 0.15 * Value],
            ]
        )

    def Rates(Value):
        return np.array([[0.1, 0.15], [0.1, -0.15]])

    Model = FieldOfViewRelay(Targets, Rates, **MODEL_PARAMETERS)
    Initial = np.vstack([np.zeros(2), Targets(0.0)])
    Time, Motion = Simulate(Model.SysDyn, Initial, **SIMULATION_PARAMETERS)
    Relative = Motion[:, 1:] - Motion[:, :1]
    Angles = np.arctan2(Relative[:, :, 1], Relative[:, :, 0])
    Metadata.update(
        Kind="fovrelay", Angles=Angles, HalfAngle=np.array([Model.HalfAngle])
    )
    VisualizerInstance = Visualizer(
        "Field-of-View Relay Control", Time, Motion, Metadata
    )
    VisualizerInstance.PrintResults()
    VisualizerInstance.Save(OUTPUT_DIRECTORY)


if __name__ == "__main__":
    main()
