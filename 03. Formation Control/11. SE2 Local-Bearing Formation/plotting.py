"""Plots and saved numerical results for SE2 Local-Bearing Formation."""

import os

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


class Visualizer:
    def __init__(self, Title, Time, Motion, Metadata):
        self.Title = Title
        self.Time = Time
        self.Motion = Motion
        self.Metadata = Metadata

    def Error(self):
        return np.sqrt(2 * self.Metadata["BearingPotential"] / (2 * len(self.Metadata["Edges"])))

    def PrintResults(self):
        Error = self.Error()
        print(f"{self.Title}: initial_error={Error[0]:.4f}, final_error={Error[-1]:.4f}")

    def CreatePlotsFigure(self):
        Figure, Axis = plt.subplots(figsize=(9, 4.4), layout="constrained")
        Axis.plot(self.Time, self.Error(), lw=2.3, label="Disagreement / tracking error")
        Axis.set(
            xlabel=self.Metadata.get("TimeLabel", "Time (s)"),
            ylabel="Error / progress metric",
            title=self.Title,
        )
        Axis.grid(alpha=0.3)
        Axis.legend(loc="upper right")
        return Figure

    def CreateTrajectoryFigure(self):
        Motion = self.Motion
        Figure = plt.figure(figsize=(8.5, 6), layout="constrained")
        Axis = Figure.add_subplot(111)
        Path = Motion[:, :, :2]
        for AgentIndex in range(Path.shape[1]):
            Axis.plot(Path[:, AgentIndex, 0], Path[:, AgentIndex, 1], lw=1.6)
            Axis.scatter(*Path[0, AgentIndex], marker="o", s=18)
            Axis.scatter(*Path[-1, AgentIndex], marker="x", s=65)
        Final = Path[-1]
        for One, Two in self.Metadata["Edges"]:
            Axis.plot(Final[[One, Two], 0], Final[[One, Two], 1], color="gray", lw=0.8, alpha=0.45)
        Axis.margins(0.18)
        Heading = Motion[-1, :, 2]
        Axis.quiver(
            Path[-1, :, 0],
            Path[-1, :, 1],
            np.cos(Heading),
            np.sin(Heading),
            angles="xy",
            scale_units="xy",
            scale=4.0,
            width=0.005,
            color="gray",
        )
        Labels = self.Metadata.get("AxisLabels", ("X", "Y"))
        Axis.set(xlabel=Labels[0], ylabel=Labels[1], title=self.Title)
        Axis.set_aspect("equal", adjustable="box")
        Axis.grid(alpha=0.22)
        return Figure

    def Save(self, OutputDir):
        os.makedirs(OutputDir, exist_ok=True)
        Figure = self.CreatePlotsFigure()
        Figure.savefig(os.path.join(OutputDir, "tracking_plots.svg"))
        plt.close(Figure)
        Figure = self.CreateTrajectoryFigure()
        Figure.savefig(os.path.join(OutputDir, "trajectory.svg"))
        plt.close(Figure)
        Arrays = {"Time": self.Time, "Motion": self.Motion, "Error": self.Error()}
        for Name, Value in self.Metadata.items():
            if isinstance(Value, np.ndarray):
                Arrays[Name] = Value
        np.savez_compressed(os.path.join(OutputDir, "simulation_data.npz"), **Arrays)
