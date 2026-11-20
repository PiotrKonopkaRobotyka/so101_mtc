# so101_mtc in a container: ROS 2 Jazzy desktop (RViz), the packages' apt dependencies through rosdep, the
# third-party models (tools/fetch_models.py) and the built workspace in /ws. CI builds this image and runs
# `colcon test` in it (.github/workflows/tests.yml). From this repository's root:
#
#   docker build -t so101_mtc .            # downloads about 2.5 GB, builds in about 10 minutes
#   xhost +local:docker                    # lets the container open windows on your display
#   docker run -it --rm --net=host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix so101_mtc
#
# The last command starts the simulation and RViz (sim.launch.py); append a command to run something else, e.g.
# `bash` for a shell with the workspace sourced.
FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /ws

COPY . src/so101_mtc

# MoveIt 2, MTC, mujoco_ros2_control and the rest from apt, as in the native setup (ADR 0002); git for the models.
# The upgrade first: the base image is an older sync, and ROS packages mix only within one sync (ros2_control's ABI)
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y --no-install-recommends git \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src --rosdistro jazzy -y \
    && rm -rf /var/lib/apt/lists/*

# the pinned folders of SO-ARM100 and MuJoCo Menagerie, into src/ next to this repository
RUN python3 src/so101_mtc/tools/fetch_models.py

# two packages at a time: the C++ of MoveIt and MTC needs about 2 GB of memory per compiler
RUN source /opt/ros/jazzy/setup.bash \
    && MAKEFLAGS="-j2" colcon build --parallel-workers 2 \
    && echo "source /ws/install/setup.bash" >> /root/.bashrc

CMD ["bash", "-c", "source /ws/install/setup.bash && ros2 launch so101_mtc_bringup sim.launch.py"]
