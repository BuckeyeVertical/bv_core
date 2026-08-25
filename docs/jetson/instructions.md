# Jetson Mission Computer Setup

This recreates the validated Buckeye Vertical mission computer on a Jetson Orin Nano. Bevy and Gazebo do not run on this machine.

The tested platform and artifact hashes are in [platform.md](platform.md).

## 1. Save required artifacts

Keep these outside the Jetson before flashing it:

- `ltdetr.pt`
- `torch-2.8.0-cp310-cp310-linux_aarch64.whl`
- `torchvision-0.23.0-cp310-cp310-linux_aarch64.whl`
- The built `bv_gcs/web/dist` directory
- Field versions of `mission_params.yaml` and `vision_params.yaml`
- SSH keys or other machine-specific credentials

The two PyTorch wheels are the verified CUDA builds. Do not replace them with the generic PyPI ARM wheels; the tested generic `torch==2.8.0` wheel was CPU-only.

## 2. Flash JetPack

Flash the Orin Nano with [JetPack 6.2.1](https://developer.nvidia.com/embedded/jetpack-sdk-621) using its SD-card image or NVIDIA SDK Manager.

Use Ubuntu 22.04 and create the `bvorinnano` user. If a different username is used, update absolute model and wheel paths in the configuration and lock files.

After first boot:

```bash
sudo apt update
sudo apt upgrade -y
sudo nvpmodel -m 2
sudo timedatectl set-timezone America/New_York
sudo reboot
```

Mode `2` is `MAXN_SUPER` on the validated Orin Nano image. Confirm it after reboot with `sudo nvpmodel -q`.

Do not install a generic Ubuntu kernel or desktop NVIDIA driver. JetPack supplies the matched Tegra kernel, firmware, CUDA runtime, cuDNN, and TensorRT packages.

Verify the base image:

```bash
cat /etc/nv_tegra_release
uname -r
dpkg -l nvidia-l4t-core nvidia-l4t-kernel nvidia-l4t-cuda
```

## 3. Install ROS 2 Humble

Follow the official [ROS 2 Humble Ubuntu instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). The concise repository setup is:

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F 'tag_name' | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

Clone the source now so the curated package manifest is available:

```bash
sudo apt install -y git
mkdir -p ~/bv_ws/src
cd ~/bv_ws/src
git clone https://github.com/BuckeyeVertical/bv_msgs.git
git clone https://github.com/BuckeyeVertical/bv_core.git
git clone https://github.com/BuckeyeVertical/bv_gcs.git
```

Install the curated system package set:

```bash
cd ~/bv_ws/src/bv_core/docs/jetson
sudo apt install -y $(grep -Ev '^[[:space:]]*(#|$)' apt-packages.txt)
```

Prevent ModemManager from claiming the PX4 serial device:

```bash
sudo apt purge -y modemmanager
sudo usermod -aG dialout,video,render "$USER"
```

Install the MAVROS GeographicLib data:

```bash
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

Log out and back in after changing groups.

## 4. Restore deployment artifacts

```bash
mkdir -p ~/bv_ws/wheels
```

For an exact rebuild, check out the tested commits listed in [platform.md](platform.md). Otherwise use the current reviewed `main` branches.

Restore the model and CUDA wheel bundle:

```bash
cp /path/to/ltdetr.pt ~/bv_ws/src/bv_core/ltdetr.pt
cp /path/to/torch-2.8.0-cp310-cp310-linux_aarch64.whl ~/bv_ws/wheels/
cp /path/to/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl ~/bv_ws/wheels/
cp -a /path/to/bv_gcs-dist ~/bv_ws/src/bv_gcs/web/dist

cd ~/bv_ws/wheels
cp ~/bv_ws/src/bv_core/docs/jetson/wheels.sha256 SHA256SUMS
sha256sum -c SHA256SUMS
echo '1061cc78fcbc70fca4a4a24bca9902bfd30d50a7f653864554427604e51b7bf2  ../src/bv_core/ltdetr.pt' | sha256sum -c -
```

Build the GCS frontend on a workstation when no saved bundle is available, then copy it to the Jetson. Node is not required on the flight computer:

```bash
cd /path/to/workstation/bv_gcs/web
npm ci
npm run build
rsync -az dist/ bvorinnano@JETSON_IP:~/bv_ws/src/bv_gcs/web/dist/
```

Copy the field configuration into `bv_core/config`. Confirm that `ml_model_path` points to the restored model.

## 5. Create the Python environment

ROS and the custom system OpenCV must remain visible, while old per-user pip packages must not be visible.

```bash
cd ~/bv_ws
python3 -m venv --system-site-packages .venv
source .venv/bin/activate

python -m pip install pip==25.3 setuptools==79.0.1 wheel==0.48.0
cp src/bv_core/docs/jetson/usercustomize.py .venv/lib/python3.10/site-packages/

python -m pip install --no-deps \
  wheels/torch-2.8.0-cp310-cp310-linux_aarch64.whl \
  wheels/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl
python -m pip install --no-deps -r src/bv_core/docs/jetson/python-requirements.txt

cp src/bv_core/docs/jetson/colcon .venv/bin/colcon
chmod 755 .venv/bin/colcon
```

`--no-deps` is intentional. It prevents pip from adding a non-GStreamer OpenCV wheel and keeps the resolved package graph fixed.

## 6. Build OpenCV with GStreamer

The real camera pipeline requires OpenCV with GStreamer enabled. Pip OpenCV wheels do not satisfy this setup.

```bash
mkdir -p ~/src
cd ~/src
git clone --depth 1 --branch 4.10.0 https://github.com/opencv/opencv.git

cmake -S opencv -B opencv-build \
  -D CMAKE_BUILD_TYPE=Release \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D WITH_GSTREAMER=ON \
  -D WITH_V4L=ON \
  -D WITH_CUDA=OFF \
  -D BUILD_TESTS=OFF \
  -D BUILD_PERF_TESTS=OFF \
  -D BUILD_EXAMPLES=OFF \
  -D BUILD_opencv_python3=ON \
  -D PYTHON3_EXECUTABLE="$HOME/bv_ws/.venv/bin/python" \
  -D PYTHON3_PACKAGES_PATH=/usr/local/lib/python3.10/dist-packages

cmake --build opencv-build -j2
sudo cmake --install opencv-build
sudo ldconfig
```

Verify that the correct build wins:

```bash
source ~/bv_ws/.venv/bin/activate
python - <<'PY'
import cv2

print(cv2.__version__)
print(cv2.__file__)
print("GStreamer: YES" if "GStreamer:                   YES" in cv2.getBuildInformation() else "GStreamer: NO")
PY
```

Expected: OpenCV `4.10.0`, a path under `/usr/local`, and `GStreamer: YES`.

Do not install `opencv-python` or `opencv-python-headless`. `pip check` will report those names as missing because the working OpenCV is system-installed; that metadata warning is expected.

## 7. Configure shell startup

Add this block to `~/.bashrc`:

```bash
# BV mission environment
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:${LD_LIBRARY_PATH:-}
source /opt/ros/humble/setup.bash
source "$HOME/bv_ws/.venv/bin/activate"
test -f "$HOME/bv_ws/install/setup.bash" && source "$HOME/bv_ws/install/setup.bash"
```

Open a new shell and verify:

```bash
command -v python
command -v colcon
```

Both paths should begin with `~/bv_ws/.venv/bin/`.

## 8. Build and test

```bash
cd ~/bv_ws
colcon build
source install/setup.bash

python -m pytest -q src/bv_core/test src/bv_gcs/test \
  --ignore=src/bv_core/test/test_flake8.py \
  --ignore=src/bv_core/test/test_pep257.py
```

Validated result: `308 passed, 1 skipped`.

Check CUDA, ROS, and OpenCV together:

```bash
python - <<'PY'
import cv2
import rclpy
import torch

assert torch.cuda.is_available()
assert "GStreamer:                   YES" in cv2.getBuildInformation()
print(torch.__version__, torch.version.cuda, torch.cuda.get_device_name(0))
print(cv2.__version__, cv2.__file__)
print(rclpy.__file__)
PY
```

Test the model wrapper without a camera:

```bash
python - <<'PY'
import numpy as np
from bv_core.detectors.ml_detector import MLDetector

detector = MLDetector(
    "/home/bvorinnano/bv_ws/src/bv_core/ltdetr.pt",
    (1920, 1920),
    0.2,
)
print(len(detector.process_frame(np.zeros((640, 640, 3), dtype=np.uint8))))
PY
```

## 9. Test GCS

```bash
ros2 launch bv_gcs gcs.launch.py
```

In another terminal:

```bash
curl -I http://127.0.0.1:8765/
```

Expected: `HTTP/1.1 200 OK`.

When `Approval_required: false`, `mission.launch.py` does not start GCS automatically. Run it separately if telemetry or live preview is still wanted.

## 10. Connect hardware

Camera:

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext

gst-launch-1.0 -v \
  v4l2src device=/dev/video0 num-buffers=8 ! \
  image/jpeg,width=4640,height=3480,framerate=8/1 ! \
  jpegdec ! fakesink
```

PX4:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
ros2 launch mavros px4.launch fcu_url:=serial:///dev/ttyACM0:57600
```

Confirm the baud rate and device name against the PX4 MAVLink port configuration. Then verify:

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/global_position/global --once
```

Finally:

```bash
ros2 launch bv_core mission.launch.py
```

## Avoid

- Do not install Bevy, Gazebo, Node, Foxglove, or PX4 SITL on the flight computer.
- Do not install pip OpenCV.
- Do not install a generic CPU PyTorch wheel.
- Do not run `apt autoremove` after JetPack package changes without reviewing every proposed removal.
- Do not validate a flight configuration for the first time with props installed.

## Troubleshooting

If LTDETR reports an NvMap allocation failure while several gigabytes remain available, reboot before changing packages. Builds and large file copies can fragment the Jetson's shared memory. The validated model loads in about eight seconds and occupies about 887 MB of CUDA memory.
