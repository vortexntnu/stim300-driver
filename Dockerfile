FROM ros:kinetic-ros-core-xenial
WORKDIR /catkin_ws
RUN apt-get update
RUN apt-get install --yes python-catkin-tools build-essential
COPY . ./src/stim300

# See https://stackoverflow.com/questions/20635472/using-the-run-instruction-in-a-dockerfile-with-source-does-not-work
RUN ["/bin/bash", "-c", "source /opt/ros/kinetic/setup.bash && \
    catkin build && \
    catkin run_tests driver_stim300 --no-deps"]
