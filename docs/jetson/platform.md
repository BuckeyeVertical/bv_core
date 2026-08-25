# Validated Jetson Platform

Snapshot date: 2026-08-24.

These are validation targets, not individual apt version pins. Kernel, firmware, CUDA, cuDNN, and TensorRT must come from one compatible JetPack/L4T release.

## Platform

| Component | Validated value |
| --- | --- |
| Hardware | Jetson Orin Nano Super |
| Architecture | aarch64 |
| Ubuntu | 22.04.5 LTS |
| JetPack family | 6.2.1 |
| L4T | R36.4.7 |
| Kernel | 5.15.148-tegra |
| Power mode | MAXN_SUPER |
| CUDA runtime | 12.6.68 |
| cuDNN | 9.3.0.75 |
| TensorRT | 10.3.0.30 |
| GStreamer | 1.20.3 |
| OpenCV | 4.10.0, GStreamer YES, V4L2 YES, CUDA OFF |
| ROS | ROS 2 Humble on Ubuntu 22.04 |
| MAVROS | 2.14.0 |
| Python | 3.10 |
| PyTorch | 2.8.0, CUDA 12.6 |
| Torchvision | 0.23.0 |

The validated machine has CUDA runtime and development headers, not the full `nvcc` compiler toolkit. The mission does not compile CUDA code on the Jetson.

The GCS preview uses the tested `x264enc` software path. `nvv4l2h264enc` was not present on this Orin Nano installation.

## NVIDIA packages

```text
nvidia-l4t-core   36.4.7-20250918154033
nvidia-l4t-kernel 5.15.148-tegra-36.4.7-20250918154033
nvidia-l4t-cuda   36.4.7-20250918154033
cuda-cudart-12-6  12.6.68-1
cuda-cupti-12-6   12.6.68-1
libcublas-12-6    12.6.1.4-1
libcudnn9-cuda-12 9.3.0.75-1
libnvinfer10      10.3.0.30-1+cuda12.5
```

Do not install a generic Ubuntu kernel or desktop NVIDIA driver over these packages.

## Repository snapshot

```text
bv_core b0d89b5840976c21268779fe8c6fb137d4441c04
bv_msgs 296530e2c99d0dddce0c45f855d513755af4842f
bv_gcs  3a3c406d724532f5b925341c30cac0088a2aa57f
```

## Required artifacts

```text
1061cc78fcbc70fca4a4a24bca9902bfd30d50a7f653864554427604e51b7bf2  ltdetr.pt
152f3dca8af3f8ac0bef33d71fe4a1d748fb8c04fecc46838692b83777c6b1ea  torch-2.8.0-cp310-cp310-linux_aarch64.whl
51610f1e93d2db2c031da5c5f28d45054ae2800ecdbb5723e3d8371c56d3efbf  torchvision-0.23.0-cp310-cp310-linux_aarch64.whl
```

The model and CUDA wheels are deployment artifacts and are not stored in normal Git history. Back them up before reimaging the machine.

## Driver and service state

- `ModemManager` is removed so it cannot claim the PX4 serial device.
- The user belongs to `dialout`, `video`, and `render`.
- MAVROS GeographicLib includes `/usr/share/GeographicLib/geoids/egm96-5.pgm`.
- The camera uses V4L2 and MJPEG through GStreamer.
- No custom camera kernel module is required for a standard USB UVC camera.

## Verification commands

```bash
cat /etc/nv_tegra_release
uname -r
dpkg -l nvidia-l4t-core nvidia-l4t-kernel nvidia-l4t-cuda
dpkg -l cuda-cudart-12-6 cuda-cupti-12-6 libcudnn9-cuda-12 libnvinfer10
sudo nvpmodel -q
```

Official references:

- [NVIDIA JetPack 6.2.1](https://developer.nvidia.com/embedded/jetpack-sdk-621)
- [NVIDIA JetPack installation](https://docs.nvidia.com/jetson/jetpack/install-setup/index.html)
- [NVIDIA PyTorch for Jetson](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)
- [ROS 2 Humble Ubuntu installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [MAVROS Humble documentation](https://docs.ros.org/en/humble/p/mavros/)
