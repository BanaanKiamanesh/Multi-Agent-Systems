"""Plots and saved numerical results for Vectorized Obstacle Flocking."""

import os

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle


class Visualizer:
    def __init__(self, Title, Time, Motion, Metadata):
        self.Title = Title
        self.Time = Time
        self.Motion = Motion
        self.Metadata = Metadata

    def Error(self):
        Motion = self.Motion
        Position, _Velocity = (Motion[:, :, 0], Motion[:, :, 1])
        return np.mean(np.linalg.norm(Position - self.Metadata["Goal"], axis=2), axis=1)

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
        OtherAxis = Axis.twinx()
        OtherAxis.plot(
            self.Time,
            self.Metadata["MeanDegree"],
            "--",
            color="gray",
            lw=1.2,
            label="Mean neighbor degree",
        )
        OtherAxis.set_ylabel("Mean neighbor degree")
        OtherAxis.legend(loc="center right")
        Axis.legend(loc="upper right")
        return Figure

    def CreateTrajectoryFigure(self):
        Motion = self.Motion
        Figure = plt.figure(figsize=(8.5, 6), layout="constrained")
        if Motion.shape[-1] == 3:
            Axis = Figure.add_subplot(111, projection="3d")
            Path = Motion[:, :, 0]
            for AgentIndex in range(Path.shape[1]):
                Axis.plot(*Path[:, AgentIndex].T, lw=2.1)
                Axis.scatter(*Path[0, AgentIndex], marker="o")
                Axis.scatter(*Path[-1, AgentIndex], marker="x")
            Axis.scatter(*self.Metadata["Goal"], marker="*", s=100, color="black", label="Goal")
            Theta, Phi = np.meshgrid(np.linspace(0, 2 * np.pi, 24), np.linspace(0, np.pi, 14))
            for CenterX, CenterY, CenterZ, Radius in self.Metadata["Obstacles"]:
                Axis.plot_wireframe(
                    CenterX + Radius * np.cos(Theta) * np.sin(Phi),
                    CenterY + Radius * np.sin(Theta) * np.sin(Phi),
                    CenterZ + Radius * np.cos(Phi),
                    color="gray",
                    alpha=0.3,
                )
            Axis.set_box_aspect(np.ptp(Path.reshape(-1, 3), axis=0) + 0.2)
            if False or True:
                Axis.legend(loc="best")
            Axis.set(xlabel="X", ylabel="Y", zlabel="Z", title=self.Title)
            return Figure
        Axis = Figure.add_subplot(111)
        Path = Motion[:, :, 0]
        for AgentIndex in range(Path.shape[1]):
            Axis.plot(Path[:, AgentIndex, 0], Path[:, AgentIndex, 1], lw=1.6)
            Axis.scatter(*Path[0, AgentIndex], marker="o", s=18)
            Axis.scatter(*Path[-1, AgentIndex], marker="x", s=65)
        Adjacency = self.Metadata["Adjacency"][-1]
        for One, Two in zip(*np.where(np.triu(Adjacency, 1))):
            Axis.plot(
                Path[-1, [One, Two], 0], Path[-1, [One, Two], 1], color="gray", alpha=0.3, lw=0.7
            )
        Axis.scatter(*self.Metadata["Goal"], marker="*", color="black", s=100, label="Goal")
        for CenterX, CenterY, Radius in self.Metadata["Obstacles"]:
            Axis.add_patch(Circle((CenterX, CenterY), Radius, fc="slategray", alpha=0.4))
        if False or True:
            Axis.legend(loc="best")
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

        if self.Motion.shape[-1] != 2:
            return
        Path = self.Motion[:, :, 0]
        Figure, Axis = plt.subplots(figsize=(5.5, 5.5))
        for CenterX, CenterY, Radius in self.Metadata["Obstacles"]:
            Axis.add_patch(Circle((CenterX, CenterY), Radius, fc="gray", alpha=0.4))
        Axis.scatter(*self.Metadata["Goal"], marker="*", color="black", s=70)
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
