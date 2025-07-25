FROM osrf/ros:noetic-desktop-full
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

# Install prerequisites
RUN apt-get update && apt-get install -y \
    wget gnupg lsb-release apt-transport-https

# Add external package sources
RUN echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/ros/ubuntu focal main" \
      > /etc/apt/sources.list.d/tmc.list && \
    echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/tmc/ubuntu focal multiverse main" \
      >> /etc/apt/sources.list.d/tmc.list && \
    echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable focal main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

# Add GPG keys
RUN wget https://hsr-user:jD3k4G2e@packages.hsr.io/tmc.key -O - | apt-key add - && \
    wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -O - | apt-key add - && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

# Auth config for packages.hsr.io
RUN mkdir -p /etc/apt/auth.conf.d && \
    echo -e "machine packages.hsr.io\nlogin hsr-user\npassword jD3k4G2e" > /etc/apt/auth.conf.d/auth.conf

# Package pinning
#RUN echo -e "Package: ros-noetic-laser-ortho-projector\nPin: version 0.3.3*\nPin-Priority: 1001\n\n\
      #Package: ros-noetic-laser-scan-matcher\nPin: version 0.3.3*\nPin-Priority: 1001\n\n\
      #Package: ros-noetic-laser-scan-sparsifier\nPin: version 0.3.3*\nPin-Priority: 1001\n\n\
      #Package: ros-noetic-laser-scan-splitter\nPin: version 0.3.3*\nPin-Priority: 1001\n\n\
      #Package: ros-noetic-ncd-parser\nPin: version 0.3.3*\nPin-Priority: 1001\n\n\
      #Package: ros-noetic-polar-scan-matcher\nPin: version 0.3.3*\nPin-Priority: 1001\n\n\
      #Package: ros-noetic-scan-to-cloud-converter\nPin: version 0.3.3*\nPin-Priority: 1001\n\n\
      #Package: ros-noetic-scan-tools\nPin: version 0.3.3*\nPin-Priority: 1001" > /etc/apt/preferences

RUN printf '%s\n' \
  'Package: ros-noetic-laser-ortho-projector' \
  'Pin: version 0.3.3*' \
  'Pin-Priority: 1001' \
  '' \
  'Package: ros-noetic-laser-scan-matcher' \
  'Pin: version 0.3.3*' \
  'Pin-Priority: 1001' \
  '' \
  'Package: ros-noetic-laser-scan-sparsifier' \
  'Pin: version 0.3.3*' \
  'Pin-Priority: 1001' \
  '' \
  'Package: ros-noetic-laser-scan-splitter' \
  'Pin: version 0.3.3*' \
  'Pin-Priority: 1001' \
  '' \
  'Package: ros-noetic-ncd-parser' \
  'Pin: version 0.3.3*' \
  'Pin-Priority: 1001' \
  '' \
  'Package: ros-noetic-polar-scan-matcher' \
  'Pin: version 0.3.3*' \
  'Pin-Priority: 1001' \
  '' \
  'Package: ros-noetic-scan-to-cloud-converter' \
  'Pin: version 0.3.3*' \
  'Pin-Priority: 1001' \
  '' \
  'Package: ros-noetic-scan-tools' \
  'Pin: version 0.3.3*' \
  'Pin-Priority: 1001' \
  '' > /etc/apt/preferences 

# Set non-interactive mode to prevent keyboard config prompts
ENV DEBIAN_FRONTEND=noninteractive

# Update and install the HSR desktop
RUN apt-get update && apt-get install -y \
     ros-noetic-tmc-desktop-full

RUN apt-get update && apt-get install -y \
    qtbase5-dev \
    libprotobuf-dev \
    protobuf-compiler

RUN apt-get update && apt-get install -y \
    gazebo11 \
    libgazebo11-dev \
    ros-noetic-gazebo-ros-pkgs

RUN apt-get update && apt-get install -y \
    ros-noetic-xacro


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
RUN git clone https://github.com/SUTURO/hsr_description.git
RUN git clone https://github.com/hsr-project/hsrb_rosnav.git
RUN git clone --branch robocup https://github.com/SUTURO/suturo_resources.git

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
