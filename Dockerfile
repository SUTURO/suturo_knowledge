FROM ros:noetic-ros-core
MAINTAINER Sascha Jongebloed, jongebloed@uni-bremen.de

ENV SWI_HOME_DIR=/usr/lib/swi-prolog
ENV LD_LIBRARY_PATH=/usr/lib/swi-prolog/lib/x86_64-linux:$LD_LIBRARY_PATH

# This steps seem to be necessary for now, TODO: check if this is still needed
# ─── STEP A: Remove any pre-existing ROS list files ─────────────────────────────────
RUN rm -f /etc/apt/sources.list.d/*ros*.list
# ─── STEP B: Update Ubuntu repos, install curl and gnupg2, and fetch the new ROS key ──
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release && \
    mkdir -p /usr/share/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
# ─── STEP C: Recreate ros-latest.list so it points at our new keyring, then update ───
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros/ubuntu focal main" \
      > /etc/apt/sources.list.d/ros-latest.list && \
      apt-get update
    
RUN apt-get update && apt-get install -y \
    software-properties-common && \
    apt-add-repository ppa:swi-prolog/stable && \
    apt-get update && apt-get install -y \
    gdb \
    g++ \
    clang \
    cmake \
    make \
    libeigen3-dev \
    libspdlog-dev \
    libraptor2-dev \
    librdf0-dev \
    libgtest-dev \
    libboost-python-dev \
    libboost-serialization-dev \
    libboost-program-options-dev \
    libfmt-dev \
    mongodb-clients \
    libmongoc-1.0-0 \
    libmongoc-dev \
    doxygen \
    graphviz \
    python3 \
    python3-dev \
    python3-pip \
    python3-venv \
    python-is-python3 \
    python3-catkin-pkg \
    python3-catkin-tools \
    git \
    ros-noetic-catkin \
    ros-noetic-tf \
    swi-prolog* \
    libjson-glib-dev \
    ros-noetic-urdf

RUN mkdir /catkin_ws
RUN mkdir /catkin_ws/src

# Install MongoDB Community Edition
# Source: https://www.mongodb.com/docs/manual/tutorial/install-mongodb-on-ubuntu/#std-label-install-mdb-community-ubuntu
ARG MONGODEB_VERSION=4.4
RUN curl -fsSL https://www.mongodb.org/static/pgp/server-${MONGODEB_VERSION}.asc | \
    gpg -o /usr/share/keyrings/mongodb-server-${MONGODEB_VERSION}.gpg --dearmor
RUN echo "deb [ arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server-${MONGODEB_VERSION}.gpg ] https://repo.mongodb.org/apt/ubuntu focal/mongodb-org/${MONGODEB_VERSION} multiverse" | \
    tee /etc/apt/sources.list.d/mongodb-org-${MONGODEB_VERSION}.list
RUN apt update
RUN apt-get update \
 && apt-get install -y \
      mongodb-org-server \
      mongodb-org-shell \
      mongodb-org-mongos \
 && rm -rf /var/lib/apt/lists/*
RUN mkdir -p /data/db && \
    chown -R ${NB_USER}:users /data/db

# Build workspace with knowrob
WORKDIR /catkin_ws/src
RUN git clone https://github.com/SUTURO/knowrob.git
RUN git clone https://github.com/SUTURO/rosprolog.git
RUN git clone https://github.com/code-iai/iai_common_msgs.git
WORKDIR /catkin_ws
RUN /usr/bin/catkin init
RUN . /opt/ros/noetic/setup.sh && /usr/bin/catkin build

# Build workspace with suturo_knowledge
WORKDIR /catkin_ws/src
ADD . /catkin_ws/src/suturo_knowledge
WORKDIR /catkin_ws
RUN . /opt/ros/noetic/setup.sh && /usr/bin/catkin build

COPY run_knowrob_local.sh /run_knowrob_local.sh

ENTRYPOINT ["/run_knowrob_local.sh"]
