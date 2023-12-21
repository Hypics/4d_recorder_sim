import numpy as np
from PIL import Image

from carb import log_warn
from carb.input import KeyboardEventType
from omni.isaac.core.utils import rotations
from omni.isaac.sensor import Camera


def write_rgb_data(rgb_data, file_path):
    rgb_image_data = np.frombuffer(rgb_data, dtype=np.uint8).reshape(
        *rgb_data.shape, -1
    )
    rgb_img = Image.fromarray(rgb_image_data, "RGBA")
    rgb_img.save(file_path + ".png")


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
