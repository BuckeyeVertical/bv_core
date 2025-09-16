# container/Dockerfile
FROM arm64v8/ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-lc"]

ENV DEBIAN_FRONTEND=noninteractive
ENV CXXFLAGS="-std=c++14"

# --- Base tools & build deps ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    gnupg2 curl lsb-release software-properties-common \
    python3-pip git cmake build-essential libeigen3-dev ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# --- ROS dev tools for colcon / rosdep ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions python3-rosdep python3-vcstool ros-dev-tools \
 && rosdep init || true && rosdep update \
 && rm -rf /var/lib/apt/lists/*

# --- Python tooling / project requirements ---
RUN python3 -m pip install --upgrade pip && \
    pip install --no-cache-dir -U empy==3.3.4 pyros-genmsg setuptools==65.5.1

# requirements.txt lives under container/
COPY container/requirements.txt /requirements.txt
RUN pip install --no-cache-dir -r /requirements.txt

# -------------------- ROS 2 workspace --------------------
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src
# Build context must include ./src at repo root
COPY src/ /ros2_ws/src/


RUN apt-get update && apt-get install -y --no-install-recommends \
    git cmake build-essential \
 && rm -rf /var/lib/apt/lists/* && \
    cd /tmp && \
    git clone --depth 1 https://github.com/geographiclib/geographiclib.git && \
    cd geographiclib && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j"$(nproc)" && make install && ldconfig && \
    test -f /usr/local/lib/cmake/GeographicLib/geographiclib-config.cmake

# Help CMake find it
ENV GeographicLib_DIR=/usr/local/lib/cmake/GeographicLib
ENV CMAKE_PREFIX_PATH=/usr/local:/usr:/usr/lib/aarch64-linux-gnu

RUN cd /root && \
    git clone --depth 1 --recursive https://github.com/PX4/PX4-Autopilot.git && \
    yes | bash /root/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
    




RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-mavros-msgs ros-humble-cv-bridge \
    python3-opencv python3-pil \
 && rm -rf /var/lib/apt/lists/*


RUN source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DGeographicLib_DIR=${GeographicLib_DIR}

# Entrypoint & env
COPY container/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
