"""Plots and saved numerical results for 3D Multi-Agent Obstacle Avoidance."""

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
        return np.mean(np.linalg.norm(Motion[:, :, 0] - self.Metadata["Goals"], axis=2), axis=1)

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
        Axis = Figure.add_subplot(111, projection="3d")
        Path = Motion[:, :, 0]
        for AgentIndex in range(Path.shape[1]):
            Axis.plot(*Path[:, AgentIndex].T, lw=2.1)
            Axis.scatter(*Path[0, AgentIndex], marker="o")
            Axis.scatter(*Path[-1, AgentIndex], marker="x")
        Theta, Phi = np.meshgrid(np.linspace(0, 2 * np.pi, 24), np.linspace(0, np.pi, 14))
        for Center, Radius in zip(self.Metadata["Centers"], self.Metadata["Radii"]):
            Axis.plot_wireframe(
                Center[0] + Radius * np.cos(Theta) * np.sin(Phi),
                Center[1] + Radius * np.sin(Theta) * np.sin(Phi),
                Center[2] + Radius * np.cos(Phi),
                color="gray",
                alpha=0.32,
            )
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
