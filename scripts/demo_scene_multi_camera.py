from omni.isaac.kit import SimulationApp

ENV_URL = "usd/demo/ParticleInflatableDemo.usd"
CONFIG = {"headless": False, "width": 1920, "height": 1080, "open_usd": ENV_URL}
simulation_app = SimulationApp(launch_config=CONFIG)
is_recording = False

from carb import log_warn
from carb.input import acquire_input_interface, KeyboardEventType
from omni.isaac.core.utils import extensions, rotations
from omni.isaac.sensor import Camera
from omni.appwindow import get_default_app_window
import omni.replicator.core as rep

import os
from datetime import datetime
import numpy as np
from PIL import Image
import time


def write_rgb_data(rgb_data, file_path):
    rgb_image_data = np.frombuffer(rgb_data, dtype=np.uint8).reshape(
        *rgb_data.shape, -1
    )
    rgb_img = Image.fromarray(rgb_image_data, "RGBA")
    rgb_img.save(file_path + ".png")


def sub_keyboard_event(event, *args, **kwargs) -> None:
    global is_recording

    if (
        event.type == KeyboardEventType.KEY_PRESS
        or event.type == KeyboardEventType.KEY_REPEAT
    ):
        if event.input.name == "R":
            # print("Pressed: R")
            is_recording = ~is_recording
            if is_recording:
                log_warn("[R] Start Recording!!")
            else:
                log_warn("[R] Stop Recording!!")
    # elif event.type == KeyboardEventType.KEY_RELEASE:
    #     if event.input.name == "R":
    #         print("Released: R")

    return True


def create_camera(
    index: int,
    width: int,
    height: int,
    position: list,
    orientation: list,
) -> Camera:
    rotation_matrix_coordinates = [
        [0.0, -1.0, 0.0],
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
    ]

    print("=====target(ZYX)=====")
    print(orientation)
    print()

    print("=====target(XYZ)=====")
    rotation_matrix_target = rotations.euler_to_rot_matrix(
        orientation, degrees=True, extrinsic=False
    )
    rotation_matrix_target = rotation_matrix_target.T
    rotation_matrix_target[0][2] *= -1
    rotation_matrix_target[1][2] *= -1
    rotation_matrix_target[2][0] *= -1
    rotation_matrix_target[2][1] *= -1
    print(rotation_matrix_target)
    print(rotations.matrix_to_euler_angles(rotation_matrix_target) / np.pi * 180.0)
    print()

    print("=====isaac=====")
    rotation_matrix_isaac = rotation_matrix_target @ rotation_matrix_coordinates
    print(rotation_matrix_isaac)
    print(rotations.matrix_to_euler_angles(rotation_matrix_isaac) / np.pi * 180.0)
    orientation_isaac = rotations.rot_matrix_to_quat(rotation_matrix_isaac)
    print(orientation_isaac)
    print()

    camera = Camera(
        prim_path="/World/Camera" + str(index),
        name="Camera" + str(index),
        position=np.array(position),
        frequency=120,
        resolution=(width, height),
        orientation=orientation_isaac,
    )

    focal_length = 20.0
    camera.set_focal_length(focal_length / 10.0)

    return camera


if __name__ == "__main__":
    if CONFIG["headless"] is True:
        extensions.enable_extension("omni.kit.livestream.native")

    acquire_input_interface().subscribe_to_keyboard_events(
        get_default_app_window().get_keyboard(), sub_keyboard_event
    )

    max_camera_plane = 6 # 6*3 = 18
    distance = 300.0
    rgb_annot_list = []

    camera_idx = 0
    height_list = [100.0, 200.0, 300.0]
    for height in height_list:
        for idx in range(0, max_camera_plane):
            pitch_angle = -np.arctan2(height, distance) / np.pi * 180.0
            azimuth_angle = camera_idx * 360.0 / max_camera_plane
            camera = rep.create.camera(
                position=(
                    distance * np.sin(azimuth_angle / 180.0 * np.pi),
                    height,
                    distance * np.cos(azimuth_angle / 180.0 * np.pi),
                ),
                look_at=(0, 0, 0),
                focal_length=20.0,
            )
            # render_product = rep.create.render_product(camera, resolution=(3840, 2160))
            # render_product = rep.create.render_product(camera, resolution=(2704, 1520))
            render_product = rep.create.render_product(camera, resolution=(1080, 720))
            rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb_annot.attach([render_product])
            rgb_annot_list.append(rgb_annot)

            camera_idx += 1

    data_dir = "data/" + datetime.now().strftime("%y%m%d_%H%M%S")
    for idx, rgb_annot in enumerate(rgb_annot_list):
        os.makedirs(data_dir + "/cam" + str(idx).zfill(2) + "/images", exist_ok=True)

    image_count = 0
    record_seconds = 1
    record_fps = 60
    while simulation_app.is_running():
        simulation_app.update()

        if is_recording:
            if image_count >= record_seconds * record_fps:
                is_recording = False
                log_warn("[R] Stop Recording!!")

            for idx, rgb_annot in enumerate(rgb_annot_list):
                rgb_data = None
                while True:
                    rgb_data = rgb_annot.get_data()
                    if rgb_data.size > 0:
                        break

                write_rgb_data(
                    rgb_data,
                    data_dir
                    + "/cam"
                    + str(idx).zfill(2)
                    + "/images/"
                    + str(image_count).zfill(4),
                )
            image_count += 1
            time.sleep(0.001)

    simulation_app.close()
