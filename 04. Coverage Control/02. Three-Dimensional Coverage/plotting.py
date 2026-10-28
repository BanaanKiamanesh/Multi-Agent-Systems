"""Plots and saved numerical results for Three-Dimensional Coverage."""

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
        Motion = self.Motion
        return np.mean(np.linalg.norm(np.diff(Motion, axis=0), axis=2), axis=1)

    def PrintResults(self):
        print(
            f"{self.Title}: cost={self.Metadata['InitialCost']:.4f} -> {self.Metadata['FinalCost']:.4f}"
        )

    def CreatePlotsFigure(self):
        Figure, Axis = plt.subplots(figsize=(9, 4.4), layout="constrained")
        Axis.plot(self.Time[1:], self.Error(), lw=2.0, label="Mean motion per sample")
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
        Axis = Figure.add_subplot(111, projection="3d")
        Path = Motion
        for AgentIndex in range(Path.shape[1]):
            Axis.plot(*Path[:, AgentIndex].T, lw=2.1)
            Axis.scatter(*Path[0, AgentIndex], marker="o")
            Axis.scatter(*Path[-1, AgentIndex], marker="x")
        Axis.set(xlabel="X", ylabel="Y", zlabel="Z", title=self.Title)
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
