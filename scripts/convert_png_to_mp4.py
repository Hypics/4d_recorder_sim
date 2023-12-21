import sys

from video_util import *

if __name__ == "__main__":
    # input value
    if len(sys.argv) > 1:
        dataset_path = sys.argv[1]
    else:
        dataset_path = "data/isaac_sim/Collisiongroups_C9H3_661"

    convert_png_to_mp4(dataset_path=dataset_path, remove_image=True)
