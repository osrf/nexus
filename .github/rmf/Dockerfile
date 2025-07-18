ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE=ros:$ROS_DISTRO-ros-base
FROM $BASE_IMAGE
ARG RMF_BRANCH=main

RUN apt-get update && apt-get install -y wget ros-dev-tools

RUN mkdir -p /rmf/src && cd /rmf \
  && wget https://raw.githubusercontent.com/open-rmf/rmf/$RMF_BRANCH/rmf.repos \
  && vcs import /rmf/src < /rmf/rmf.repos

RUN rosdep update && rosdep install --from-paths /rmf/src -yi

RUN cd /rmf \
  && . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --mixin release

RUN rm -rf /var/lib/apt/lists /rmf/build /rmf/src /rmf/log \
  && sed -i '$isource "/rmf/install/setup.bash"' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
