# syntax=docker/dockerfile:1
# Full native Gazebo application image. Isaac Sim remains a separately licensed runtime.
ARG ROS_IMAGE=ros:jazzy-ros-base
FROM ${ROS_IMAGE} AS native_environment
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG UBUNTU_APT_MIRROR=https://archive.ubuntu.com/ubuntu
RUN sed -i -E "s|https?://archive.ubuntu.com/ubuntu|${UBUNTU_APT_MIRROR}|g; s|https?://security.ubuntu.com/ubuntu|${UBUNTU_APT_MIRROR}|g" /etc/apt/sources.list.d/ubuntu.sources \
 && apt-get update -o APT::Update::Error-Mode=any -o Acquire::Retries=3 \
 && apt-get install -y --no-install-recommends -o Acquire::Retries=3 \
    ca-certificates curl build-essential cmake git patch pkg-config \
    python3-colcon-common-extensions python3-numpy python3-scipy python3-yaml python3-pil \
    libeigen3-dev libpcl-dev libopencv-dev libfmt-dev libboost-thread-dev \
    libssl-dev nlohmann-json3-dev libtinyxml2-dev libsqlite3-dev libasio-dev \
    libapr1-dev libgtest-dev \
    ros-jazzy-sophus ros-jazzy-pcl-ros ros-jazzy-pcl-conversions \
    ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-sensor-msgs-py \
    ros-jazzy-ament-cmake-auto ros-jazzy-ament-index-cpp ros-jazzy-demo-nodes-cpp \
    ros-jazzy-rclcpp-components ros-jazzy-tf2-geometry-msgs ros-jazzy-message-filters \
    ros-jazzy-rosidl-default-generators ros-jazzy-rmw-fastrtps-cpp \
    ros-jazzy-fastrtps ros-jazzy-fastcdr \
 && rm -rf /var/lib/apt/lists/*
ENV PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
WORKDIR /workspace/navigation


RUN apt-get update \
 && apt-get install -y --no-install-recommends -o Acquire::Retries=3 gnupg python3-venv python3-pip libgl1-mesa-dri libegl1 libgl1 mesa-utils ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo 'deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main' > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y --no-install-recommends -o Acquire::Retries=3 gz-harmonic libgz-sim8-dev \
 && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv --system-site-packages /opt/navigation-venv \
 && /opt/navigation-venv/bin/pip install --no-cache-dir torch==2.7.1 --index-url https://download.pytorch.org/whl/cpu
ENV NAVIGATION_ACTOR_PYTHON=/opt/navigation-venv/bin/python3 PYTHONPATH=/workspace/navigation/src

FROM native_environment AS native_builder
COPY third_party /workspace/navigation/third_party
COPY docker /workspace/navigation/docker
ARG BUILD_JOBS=2
RUN source /opt/ros/jazzy/setup.bash \
 && python3 docker/build_native.py --jobs "${BUILD_JOBS}" --execute


COPY src/embodied_navigation/localization_mapping/backend/native /workspace/navigation/src/embodied_navigation/localization_mapping/backend/native
COPY src/embodied_navigation/simulation/actuators /workspace/navigation/src/embodied_navigation/simulation/actuators
COPY src/embodied_navigation/common/communication /workspace/navigation/src/embodied_navigation/common/communication
COPY src/embodied_navigation/simulation/gazebo/robogauge/motor_src /workspace/navigation/src/embodied_navigation/simulation/gazebo/robogauge/motor_src
RUN source /opt/ros/jazzy/setup.bash \
 && CMAKE_PREFIX_PATH="/workspace/navigation/install/dds:${CMAKE_PREFIX_PATH:-}" python3 docker/build_owned_native.py --jobs "${BUILD_JOBS}" --execute

FROM native_environment AS native_runtime
COPY --from=native_builder /workspace/navigation/install /workspace/navigation/install
COPY docker/native_entrypoint.sh /workspace/navigation/docker/native_entrypoint.sh
ENTRYPOINT ["/bin/bash", "/workspace/navigation/docker/native_entrypoint.sh"]
COPY third_party /workspace/navigation/third_party
COPY docs /workspace/navigation/docs
COPY src /workspace/navigation/src
COPY configs /workspace/navigation/configs
COPY scripts /workspace/navigation/scripts
COPY tests /workspace/navigation/tests
COPY start.py pyproject.toml requirements-cpu.lock README.md /workspace/navigation/
RUN python3 -B -m unittest discover -s tests/unit -v \
 && /opt/navigation-venv/bin/python3 -B -m unittest discover -s tests/regression -v
ENV HOME=/tmp/navigation-home
RUN mkdir -p /workspace/navigation/runs && chown 1000:1000 /workspace/navigation/runs
USER 1000:1000
CMD ["python3", "/workspace/navigation/start.py", "--check"]
