"""Plots and saved numerical results for Boids Flocking 3D."""

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
        Position, Velocity = (Motion[:, :, 0], Motion[:, :, 1])
        return np.linalg.norm(Position.mean(axis=1) - self.Metadata["Reference"], axis=1) + np.mean(
            np.linalg.norm(Velocity - self.Metadata["ReferenceRates"][:, None], axis=2), axis=1
        )

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
            Axis.plot(*self.Metadata["Reference"].T, "k--", lw=1.5, label="Reference")
            Axis.set_box_aspect(np.ptp(Path.reshape(-1, 3), axis=0) + 0.2)
            if True or False:
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
        Axis.plot(*self.Metadata["Reference"].T, "k--", lw=1.3, label="Reference")
        if True or False:
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
        """Save the spatial trajectory as a GIF, including the reference or obstacles."""
        from matplotlib.animation import FuncAnimation, PillowWriter

        Path = self.Motion[:, :, 0]
        Figure = plt.figure(figsize=(6.5, 6))
        Axis = Figure.add_subplot(111, projection="3d")
        Extent = [Path.reshape(-1, 3)]
        if "Reference" in self.Metadata:
            Reference = self.Metadata["Reference"]
            Axis.plot(*Reference.T, "k--", lw=1, label="Reference")
            Extent.append(Reference)
        if "Goal" in self.Metadata:
            Goal = self.Metadata["Goal"]
            Axis.scatter(*Goal, marker="*", color="black", s=80, label="Goal")
            Extent.append(Goal[None, :])
        if "Obstacles" in self.Metadata:
            Theta, Phi = np.meshgrid(np.linspace(0, 2 * np.pi, 20), np.linspace(0, np.pi, 12))
            for X, Y, Z, Radius in self.Metadata["Obstacles"]:
                Axis.plot_wireframe(
                    X + Radius * np.cos(Theta) * np.sin(Phi),
                    Y + Radius * np.sin(Theta) * np.sin(Phi),
                    Z + Radius * np.cos(Phi),
                    color="gray",
                    alpha=0.3,
                )
                Extent.append(np.array([[X, Y, Z]]) + np.array([[-Radius], [Radius]]))
        Extent = np.vstack(Extent)
        Lower, Upper = Extent.min(axis=0) - 0.5, Extent.max(axis=0) + 0.5
        Axis.set(
            xlim=(Lower[0], Upper[0]),
            ylim=(Lower[1], Upper[1]),
            zlim=(Lower[2], Upper[2]),
            xlabel="X",
            ylabel="Y",
            zlabel="Z",
            title=self.Title,
        )
        Axis.set_box_aspect(Upper - Lower)
        if "Reference" in self.Metadata or "Goal" in self.Metadata:
            Axis.legend(loc="best")
        Lines = [Axis.plot([], [], [], lw=1.4)[0] for _ in range(Path.shape[1])]
        Points = [Axis.plot([], [], [], "o", ms=4, color=Line.get_color())[0] for Line in Lines]
        Frames = np.unique(np.linspace(0, len(Path) - 1, min(FrameNum, len(Path)), dtype=int))

        def Update(Index):
            for Agent, (Line, Point) in enumerate(zip(Lines, Points)):
                Line.set_data_3d(*Path[: Index + 1, Agent].T)
                Point.set_data_3d(*Path[Index : Index + 1, Agent].T)
            return Lines + Points

        Animation = FuncAnimation(Figure, Update, frames=Frames, blit=False, interval=50)
        try:
            Animation.save(FilePath, writer=PillowWriter(fps=20))
        finally:
            plt.close(Figure)
