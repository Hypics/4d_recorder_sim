import os
import sys

sys.path.append(".")
from utils.synology_utils import SynologySession


if __name__ == "__main__":
    # fixed value
    nas_domain = "https://ds918pluswee.synology.me"
    nas_port = 5001
    nas_user = "deepfrost"
    nas_pw = "deeeep"

    # input value
    if len(sys.argv) > 2:
        user_name = sys.argv[1]
        dataset_name = sys.argv[2]
    else:
        user_name = "isaac_sim"
        dataset_name = "Collisiongroups_C9H3_661"

    user_folder_path = f"data/{user_name}"
    user_dataset_path = f"{user_folder_path}/{dataset_name}"
    synology_output_path = f"/dataset/4DGaussians/{user_dataset_path}"

    with SynologySession(
        address=nas_domain,
        port=nas_port,
        username=nas_user,
        password=nas_pw,
        session="FileStation",
    ) as syns:
        upload_folder = user_dataset_path
        print("[upload_output] Upload outputs")
        # data/isaac_sim/Collisiongroups_C9H3_661
        print(f"[upload_output] [dataset] {upload_folder}")
        print(f"[upload_output] [destination] {synology_output_path}")
        upload_file_path_list = [
            os.path.join(upload_folder, x) for x in os.listdir(upload_folder)
        ]
        upload_response = syns.upload_files(
            path=synology_output_path,
            upload_folder=upload_folder,
            file_path_list=upload_file_path_list,
            create_parents=True,
            overwrite="overwrite",
        )
