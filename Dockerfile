FROM ros:noetic-ros-base-focal

RUN apt update && apt install --yes \
    python3-catkin-tools \
    vim \
    tmux \
    git \
    python3.8-venv \
    python3-pip \
    ros-noetic-rosserial \
    ros-noetic-rosserial-arduino \
    ros-noetic-nmea-msgs \
    rtklib \
    python3-tk \
    ros-noetic-foxglove-bridge

RUN python3 -m pip install \
    pre-commit \
    matplotlib \
    pyshp \
    utm

COPY requirements.txt /tmp
RUN python3 -m pip install -r /tmp/requirements.txt

RUN apt install ros-noetic-rviz -y

# start with `docker exec -it src-app-1 /bin/bash`
