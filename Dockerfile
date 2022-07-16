FROM ros:noetic-ros-core-focal

SHELL ["/bin/bash", "-c"] 

RUN apt-get update && apt-get install --yes --no-install-recommends python3-catkin-tools build-essential

COPY . ./imu_ws/src
RUN source /opt/ros/noetic/setup.bash && cd /imu_ws && catkin build

COPY ./entrypoint.sh /
CMD ["/entrypoint.sh"]
