ARG ROS_DISTRO=jazzy
FROM docker.io/ros:$ROS_DISTRO-ros-base
ARG NEXUS_BRANCH=main

RUN apt update && apt install -y git curl libclang-dev clang clang-tools lld

RUN mkdir -p /ws_nexus_base/src && cd /ws_nexus_base \
  && curl --output abb.repos https://raw.githubusercontent.com/osrf/nexus/refs/heads/$NEXUS_BRANCH/abb.repos \
  && vcs import /ws_nexus_base/src < abb.repos \
  && curl --output rmf.repos https://raw.githubusercontent.com/osrf/nexus/refs/heads/$NEXUS_BRANCH/rmf.repos \
  && vcs import /ws_nexus_base/src < rmf.repos

RUN rosdep update && rosdep install --ignore-src --from-paths /ws_nexus_base/src -yir

RUN cd /ws_nexus_base \
  && . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --merge-install --install-base /opt/nexus_base --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && rm -rf /ws_nexus_base

RUN rm -rf \
  /var/lib/apt/lists \
  /dist
