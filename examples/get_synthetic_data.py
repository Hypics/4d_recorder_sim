from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp(launch_config={"renderer": "RayTracedLighting", "headless": False})

import omni
import numpy as np
import os
import json
from PIL import Image
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import omni.replicator.core as rep
from omni.isaac.core.utils.semantics import add_update_semantics

# Util function to save rgb annotator data
def write_rgb_data(rgb_data, file_path):
    rgb_image_data = np.frombuffer(rgb_data, dtype=np.uint8).reshape(*rgb_data.shape, -1)
    rgb_img = Image.fromarray(rgb_image_data, "RGBA")
    rgb_img.save(file_path + ".png")


# Util function to save semantic segmentation annotator data
def write_sem_data(sem_data, file_path):
    id_to_labels = sem_data["info"]["idToLabels"]
    with open(file_path + ".json", "w") as f:
        json.dump(id_to_labels, f)
    sem_image_data = np.frombuffer(sem_data["data"], dtype=np.uint8).reshape(*sem_data["data"].shape, -1)
    sem_img = Image.fromarray(sem_image_data, "RGBA")
    sem_img.save(file_path + ".png")


# Create a new stage with the default ground plane
omni.usd.get_context().new_stage()
world = World()
world.scene.add_default_ground_plane()
world.reset()

# Run the application for several frames to allow the materials to load
for i in range(20):
    simulation_app.update()

# Create a camera and render product to collect the data from
cam = rep.create.camera(position=(3, 3, 3), look_at=(0, 0, 0))
rp = rep.create.render_product(cam, (512, 512))

# Set the output directory for the data
out_dir = os.getcwd() + "/_out_sim_get_data"
os.makedirs(out_dir, exist_ok=True)
print(f"Outputting data to {out_dir}..")

# NOTE currently replicator writers do not work correctly with isaac simulations and will interfere with the timeline
# writer = rep.WriterRegistry.get("BasicWriter")
# writer.initialize(output_dir=out_dir, rgb=True, semantic_segmentation=True, colorize_semantic_segmentation=True)
# writer.attach([rp])

# Accesing the data directly from annotators
rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
rgb_annot.attach([rp])
sem_annot = rep.AnnotatorRegistry.get_annotator("semantic_segmentation", init_params={"colorize": True})
sem_annot.attach([rp])

for i in range(5):
    cuboid = world.scene.add(DynamicCuboid(prim_path=f"/World/Cuboid_{i}", name=f"Cuboid_{i}", position=(0, 0, 10 + i)))
    add_update_semantics(cuboid.prim, "Cuboid")

    for s in range(1000):
        world.step(render=True, step_sim=True)
        vel = np.linalg.norm(cuboid.get_linear_velocity())
        if vel < 0.1:
            print(f"Cube_{i} stopped moving after {s} simulation steps, writing data..")
            # NOTE replicator's step is no longer needed since new data is fed in the annotators every world.step()
            # rep.orchestrator.step()
            write_rgb_data(rgb_annot.get_data(), f"{out_dir}/Cube_{i}_step_{s}_rgb")
            write_sem_data(sem_annot.get_data(), f"{out_dir}/Cube_{i}_step_{s}_sem")
            break

simulation_app.close()
