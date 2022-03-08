FROM ros:noetic-ros-core-focal
WORKDIR /catkin_ws
RUN apt-get update && apt-get install --yes --no-install-reccommends python3-catkin-tools build-essential
COPY . ./src/stim300

# See https://stackoverflow.com/questions/20635472/using-the-run-instruction-in-a-dockerfile-with-source-does-not-work
RUN ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
    catkin build && \
    catkin run_tests driver_stim300 --no-deps"]
