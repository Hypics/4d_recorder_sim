import os
import cv2
import shutil
from tqdm import tqdm


def convert_png_to_mp4(dataset_path: str, remove_image: bool = False):
    cam_list = sorted(os.listdir(dataset_path))
    pbar = tqdm(cam_list)
    for cam_folder_name in pbar:
        pbar.set_description(f"cam: {cam_folder_name}")
        cam_folder_path = os.path.join(dataset_path, cam_folder_name)
        img_folder_path = os.path.join(cam_folder_path, "images")

        fps = 60
        frame_array = []
        size = None

        img_list = sorted(os.listdir(img_folder_path))
        pbar2 = tqdm(img_list, leave=False)
        for img_name in pbar2:
            pbar2.set_description(f"img name: {img_name}, size: {size}")
            img = cv2.imread(os.path.join(img_folder_path, img_name))
            if size is None:
                height, width, layers = img.shape
                size = (width, height)
            frame_array.append(img)

        out = cv2.VideoWriter(
            cam_folder_path + ".mp4", cv2.VideoWriter_fourcc(*"FMP4"), fps, size
        )
        for frame in frame_array:
            out.write(frame)
        out.release()

        if remove_image:
            print(f"[convert_png_to_mp4] Remove {cam_folder_path}")
            shutil.rmtree(cam_folder_path)
