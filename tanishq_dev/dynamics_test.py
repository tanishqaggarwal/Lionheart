import tanishq_dev.dynamics_py as lionheart
from typing import Callable
import numpy as np
import vedo
import os
import tempfile
import subprocess
import tqdm


class Simulation:
    def __init__(
        self,
        initial_state: lionheart.RoverState,
        initial_position: tuple[float, float, float],
        config: lionheart.RoverConfig,
    ) -> None:
        self.rover = lionheart.Rover(initial_state, config)
        self.plotter = vedo.Plotter(interactive=False, offscreen=True)

        self.plotter.camera.SetPosition(5, 5, 5)  # Position the camera at (5,5,5)
        self.plotter.camera.SetFocalPoint(0, 0, 0)  # Look at origin
        self.plotter.camera.SetViewUp(0, 0, 1)  # Set "up" direction to Z-axis

        self.temp_dir = tempfile.TemporaryDirectory()
        self.output_folder = self.temp_dir.name

    @property
    def position(self) -> np.ndarray:
        pos = self.rover.get_position()
        return pos.to_numpy()

    @property
    def attitude(self) -> np.ndarray:
        att = self.rover.get_attitude()
        return att.to_numpy()

    def _step(
        self, dt: float, frame_count: int, collect_frame: bool, thrusts: list[float]
    ) -> None:
        # Update physics
        self.rover.update(thrusts)
        self.rover.integrate_euler(dt)

        if hasattr(self, "box"):
            self.plotter.remove(self.box)
        self.box = vedo.Box(length=1, width=1, height=1, c="blue")
        transform = np.eye(4)
        transform[:3, :3] = self.attitude
        transform[:3, 3] = self.position
        self.box.apply_transform(transform)
        self.plotter += self.box

        if collect_frame:
            frame_path = os.path.join(
                self.output_folder, f"frame_{frame_count:05d}.png"
            )
            self.plotter.screenshot(frame_path)

    def run(
        self,
        sim_time: float,
        dt: float,
        timesteps_per_frame: int,
        thrust_fn: Callable[float, list[float]],
    ) -> None:
        n_timesteps = int(sim_time // dt)
        for i in tqdm.trange(n_timesteps):
            self._step(
                dt=dt,
                frame_count=i // timesteps_per_frame,
                collect_frame=(i % timesteps_per_frame) == 0,
                thrusts=thrust_fn(t := i * dt),
            )

    def save_video(self, framerate: int, output_location: str) -> None:
        # Use ffmpeg to combine frames into video
        cmd = [
            "ffmpeg",
            "-y",  # Overwrite output file if it exists
            "-framerate",
            f"{framerate}",  # Frames per second
            "-i",
            f"{self.output_folder}/frame_%05d.png",
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            output_location,
        ]
        subprocess.run(cmd)
        print(f"Video saved at {output_location}")


config = lionheart.RoverConfig(
    water_density=1035,
    mass=1035,
    volume=1.0,
    moi=lionheart.Matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
    center_of_buoyancy=lionheart.Vector(0, 0, 0),
    thrust_positions=[
        lionheart.Vector(0, 0, 0),
        lionheart.Vector(0, 0, 0),
        lionheart.Vector(0, 0, 0),
        lionheart.Vector(0, 0, 0),
        lionheart.Vector(0, 0, 0),
    ],
    thrust_vectors=[
        lionheart.Vector(1, 0, 0),
        lionheart.Vector(1, 0, 0),
        lionheart.Vector(1, 0, 0),
        lionheart.Vector(1, 0, 0),
        lionheart.Vector(1, 0, 0),
    ],
)

initial_position = (0, 0, 0)
initial_state = lionheart.RoverState(
    position=lionheart.Vector(*initial_position),
    velocity=lionheart.Vector(0, 0, 0),
    angular_velocity=lionheart.Vector(0, 0, 1.0),
    attitude=lionheart.Matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
)
sim = Simulation(initial_state, initial_position, config)

thrust_fn = lambda t: [0, 0, 0, 0, 0]

framerate = 40
dt = 0.01
timesteps_per_frame = int(1.0 / (dt * framerate))
sim.run(
    sim_time=10.0, dt=0.01, timesteps_per_frame=timesteps_per_frame, thrust_fn=thrust_fn
)
sim.save_video(framerate, "/tmp/lionheart.mp4")
