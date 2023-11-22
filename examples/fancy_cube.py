import getpass
user = getpass.getuser()
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.utils.prims import create_prim

import numpy as np

world = World(stage_units_in_meters=1.0)
world.scene.add_ground_plane()

# Add a distant light
create_prim("/DistantLight", "DistantLight")

world.render()

fancy_cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/random_cube",
        name="fancy_cube",
        position=np.array([0, 0, 10.000]),
        scale=np.array([0.5015, 0.505, 0.5015]),
        size=1.0,
        color=np.array([0, 0, 1.0]),
    )
)
world.render()
world.reset()

for i in range(1000):
    position, orientation = fancy_cube.get_world_pose()
    linear_velocity = fancy_cube.get_linear_velocity()
    set_camera_view(eye=np.array([10.0, 0.0, 10]), target=np.array(position))
    print("Cube position is : " + str(position))
    print("Cube's orientation is : " + str(orientation))
    print("Cube's linear velocity is : " + str(linear_velocity))
    world.step(render=True)

# Cleanup application
simulation_app.close()
