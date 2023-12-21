# 4d_recorder_sim

## Introduction

4D Recorder Simulator based on [NVIDIA Isaac Sim](https://docs-prod.omniverse.nvidia.com/isaacsim/latest/index.html).

### USD Files

- demo/ParticleInflatableDemo.usd

## Getting Started

1. [Install NVIDIA Isaac Sim](https://docs-prod.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html)

    ```bash
    sudo apt update \
      && sudo apt install libfuse2
    ```

    - check IOMMU

        ```bash
        sudo dmesg | grep -e DMAR -e IOMMU
        ```

2. Set the following environment variables to your ```~/.bashrc``` file

    ```bash
    echo "export ISAACSIM_PATH='${HOME}/.local/share/ov/pkg/isaac_sim-2023.1.0-hotfix.1'" >> ~/.bashrc
    ```

3. [Install conda](https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html#macos-and-linux)

4. Create conda environment

    ```bash
    conda create -n sim python=3.10 \
      && conda activate sim \
      && cp -r conda_setup/etc $CONDA_PREFIX \
      && conda activate sim \
      && conda install tqdm \
      && pip install opencv-python
    ```

5. Verification

    ```bash
    python -c "from omni.isaac.kit import SimulationApp" \
      && python -c "import torch; print(torch.__path__)"
    ```

    - output

        ```bash
        ['~/.local/share/ov/pkg/isaac_sim-2023.1.0-hotfix.1/extscache/omni.pip.torch-1_13_1-0.1.4+104.2.lx64/torch-1-13-1/torch']
        ```

## Build and Test (with the Isaac Sim path setup above)

### Particle Inflatable Demo

```bash
python scripts/demo_scene_multi_camera.py
```

### Make dataset

- Camera cylinder set

```bash
./scripts/make_dataset.sh  cylinder
```

- Camera rectangle set

```bash
./scripts/make_dataset.sh  rectangle
```

### Convert png to mp4

```bash
python scripts/convert_png_to_mp4.py Collisiongroups_C9H3_661
```
