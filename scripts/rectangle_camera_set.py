from omni.isaac.kit import SimulationApp

# input value
import sys
if len(sys.argv) > 1:
    ENV_URL = sys.argv[1]
else:
    ENV_URL = "usd/demo/ParticleInflatableDemo.usd"

CONFIG = {"headless": False, "width": 1920, "height": 1080, "open_usd": ENV_URL}
simulation_app = SimulationApp(launch_config=CONFIG)
is_recording = False

from carb import log_warn
from carb.input import KeyboardEventType
from carb.input import acquire_input_interface
from omni.isaac.core.utils import extensions
from omni.appwindow import get_default_app_window
import omni.replicator.core as rep

import os
from datetime import datetime
import time

from omni_util import *
from video_util import *

# CAMERA_RESOLUTION = (3840, 2160)
# CAMERA_RESOLUTION = (2704, 1520)
CAMERA_RESOLUTION = (1080, 720)


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


if __name__ == "__main__":
    if CONFIG["headless"] is True:
        extensions.enable_extension("omni.kit.livestream.native")

    acquire_input_interface().subscribe_to_keyboard_events(
        get_default_app_window().get_keyboard(), sub_keyboard_event
    )

    rgb_annot_list = []

    # wall (bottom, top)
    poses_width = [-900.0, 0.0, 900.0]
    poses_depth = [-900.0, 900.0]
    poses_height = [0.0, 300.0]

    for pos_width in poses_width:
        for pos_depth in poses_depth:
            for pos_height in poses_height:
                camera = rep.create.camera(
                    position=(pos_depth, pos_height, pos_width),
                    look_at=(0, 0, 0),
                    focal_length=20.0,
                )
                render_product = rep.create.render_product(camera, resolution=CAMERA_RESOLUTION)
                rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
                rgb_annot.attach([render_product])
                rgb_annot_list.append(rgb_annot)

    # ceiling (center)
    poses_width = [-450.0, 450.0]
    poses_depth = [0.0]
    poses_height = [300.0]

    for pos_width in poses_width:
        for pos_depth in poses_depth:
            for pos_height in poses_height:
                camera = rep.create.camera(
                    position=(pos_depth, pos_height, pos_width),
                    look_at=(0, 0, 0),
                    focal_length=20.0,
                )
                render_product = rep.create.render_product(camera, resolution=CAMERA_RESOLUTION)
                rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
                rgb_annot.attach([render_product])
                rgb_annot_list.append(rgb_annot)


    data_dir = "data/isaac_sim/" + os.path.splitext(os.path.basename(ENV_URL))[0] + "_" + datetime.now().strftime("%y%m%d_%H%M%S")
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

    convert_png_to_mp4(dataset_path=data_dir, remove_image=True)
