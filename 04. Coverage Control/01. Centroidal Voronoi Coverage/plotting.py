"""Plots and saved numerical results for Centroidal Voronoi Coverage."""

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
        Axis = Figure.add_subplot(111)
        Path = Motion
        for AgentIndex in range(Path.shape[1]):
            Axis.plot(Path[:, AgentIndex, 0], Path[:, AgentIndex, 1], lw=1.6)
            Axis.scatter(*Path[0, AgentIndex], marker="o", s=18)
            Axis.scatter(*Path[-1, AgentIndex], marker="x", s=65)
        Samples = self.Metadata["Samples"]
        Axis.scatter(
            Samples[:, 0], Samples[:, 1], c=self.Metadata["Density"], s=2, alpha=0.2, cmap="viridis"
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

    def SaveAnimation(self, FilePath, FrameNum=72):
        """Create an optional compact GIF for planar trajectories (requires Pillow)."""
        from matplotlib.animation import FuncAnimation, PillowWriter

        Path = self.Motion
        Figure, Axis = plt.subplots(figsize=(5.5, 5.5))
        Min = np.min(Path.reshape(-1, 2), axis=0) - 0.7
        Max = np.max(Path.reshape(-1, 2), axis=0) + 0.7
        Axis.set(xlim=(Min[0], Max[0]), ylim=(Min[1], Max[1]), title=self.Title)
        Axis.set_aspect("equal")
        Lines = [Axis.plot([], [], lw=1.7)[0] for AgentIndex in range(Path.shape[1])]
        Points = [Axis.plot([], [], "o", ms=6)[0] for AgentIndex in range(Path.shape[1])]
        Frames = np.unique(np.linspace(0, len(Path) - 1, min(FrameNum, len(Path)), dtype=int))

        def Update(FrameIndex):
            Index = Frames[FrameIndex]
            for AgentIndex in range(Path.shape[1]):
                Lines[AgentIndex].set_data(
                    Path[: Index + 1, AgentIndex, 0], Path[: Index + 1, AgentIndex, 1]
                )
                Points[AgentIndex].set_data(
                    [Path[Index, AgentIndex, 0]], [Path[Index, AgentIndex, 1]]
                )
            return Lines + Points

        Animation = FuncAnimation(Figure, Update, frames=len(Frames), blit=True, interval=55)
        Animation.save(FilePath, writer=PillowWriter(fps=20))
        plt.close(Figure)
