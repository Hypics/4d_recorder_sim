import os
import sys
import cv2
from tqdm import tqdm

from utils import *

if __name__ == "__main__":
    # input value
    if len(sys.argv) > 1:
        dataset_path = os.path.join("data/isaac_sim", sys.argv[1])
    else:
        dataset_path = "data/isaac_sim/Collisiongroups_C9H3_661"

    convert_png_to_mp4(dataset_path=dataset_path, remove_image=False)
