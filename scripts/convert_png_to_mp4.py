import sys

sys.path.append(".")
from utils.video_utils import *

if __name__ == "__main__":
    # input value
    if len(sys.argv) > 1:
        dataset_path = sys.argv[1][:-1] if sys.argv[1][-1] == "/" else sys.argv[1]
    else:
        dataset_path = "data/isaac_sim/Collisiongroups_C9H3_661"

    convert_png_to_mp4(dataset_path=dataset_path, remove_image=True)
