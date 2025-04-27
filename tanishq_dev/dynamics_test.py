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
        rover_dimensions: tuple[float, float, float],
        config: lionheart.RoverConfig,
    ) -> None:
        self.plant = lionheart.Rover(initial_state, config)
        self.plotter = vedo.Plotter(interactive=False, offscreen=True)
        self.rover_dimensions = rover_dimensions
        self.box = None

        self.plotter.camera.SetPosition(5, 5, 5)  # Position the camera at (5,5,5)
        self.plotter.camera.SetFocalPoint(0, 0, 0)  # Look at origin
        self.plotter.camera.SetViewUp(0, 0, 1)  # Set "up" direction to Z-axis

        self.temp_dir = tempfile.TemporaryDirectory()
        self.output_folder = self.temp_dir.name

    def _step(
        self, dt: float, frame_count: int, collect_frame: bool, thrusts: list[float]
    ) -> None:
        # Update physics
        self.plant.update(thrusts)
        self.plant.integrate_rk4(dt)

        if self.box:
            self.plotter.remove(self.box)
        self.box = vedo.Box(
            length=self.rover_dimensions[0],
            width=self.rover_dimensions[1],
            height=self.rover_dimensions[2],
            c="blue",
        )
        transform = np.eye(4)
        transform[:3, :3] = self.plant.get_attitude().to_numpy()
        transform[:3, 3] = self.plant.get_position().to_numpy()
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

sim = Simulation(
    initial_state=lionheart.RoverState(
        position=lionheart.Vector(0, 0, 0),
        velocity=lionheart.Vector(0, 0, 0),
        angular_velocity=lionheart.Vector(0, 0, 1.0),
        attitude=lionheart.Matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
    ),
    rover_dimensions=(1, 1, 1),
    config=config,
)
thrust_fn = lambda t: [0, 0, 0, 0, 0]

framerate = 10
dt = 0.01
sim.run(
    sim_time=1.0,
    dt=dt,
    timesteps_per_frame=int(1.0 / (dt * framerate)),
    thrust_fn=thrust_fn,
)
sim.save_video(framerate, "/tmp/lionheart.mp4")
