import tanishq_dev.dynamics_py as lionheart
from typing import Callable
import numpy as np
import vedo

class Simulation:
    def __init__(
        self,
        initial_state: lionheart.RoverState,
        config: lionheart.RoverConfig,
        box: vedo.Box,
    ) -> None:
        self.rover = lionheart.Rover(initial_state, config)
        self.plotter = vedo.Plotter(interactive=False, axes=1)
        self.box = box
        self.plotter += self.box

    @property
    def position(self) -> np.ndarray:
        pos = self.rover.get_position()
        return pos.to_numpy()

    @property
    def attitude(self) -> np.ndarray:
        att = self.rover.get_attitude()
        return att.to_numpy()

    def _step(
        self, t: float, dt: float, thrust_fn: Callable[float, list[float]]
    ) -> None:
        # Update physics
        self.rover.update(thrust_fn(t))
        self.rover.integrate_euler(dt)

        # Update visualization
        self.box.pos(self.position)
        transform = np.eye(4)
        transform[:3, :3] = self.attitude
        transform[:3, 3] = self.position
        self.box.apply_transform(transform)
        self.plotter.render()

    def run(
        self, tf: float, dt: float, thrust_fn: Callable[float, list[float]]
    ) -> None:
        n_timesteps = int(tf // dt)
        for i in range(n_timesteps):
            self._step(i * dt, dt, thrust_fn)

    def stop(self):
        self.plotter.interactive().close()


config = lionheart.RoverConfig(
    mass=1.0,
    volume=1.0,
    moi=lionheart.Matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
    cb=lionheart.Vector(0, 0, 0),
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
    angular_velocity=lionheart.Vector(0, 0, 0),
    attitude=lionheart.Matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
)
rover_body = vedo.Box(pos=initial_position, length=1, width=1, height=1, c="blue")
sim = Simulation(initial_state, config, rover_body)

thrust_fn = lambda t: [0, 0, 0, 0, 0]
sim.run(10, 0.01, thrust_fn)
input("Press Enter to exit.")
sim.stop()
