FROM ubuntu:20.04 as communication_link_builder
# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone \
    && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
    && apt-get update && apt-get install -q -y tzdata \
    && rm -rf /var/lib/apt/lists/*
# Install ROS 2
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list
# Install build dependencies
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    build-essential \
    fakeroot \
    git-core \
    golang \
    ros-foxy-ros-base \
    libgstreamer1.0-0 \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    && rm -rf /var/lib/apt/lists/*

# Install px4-msgs
RUN curl -fsSL https://github.com/tiiuae/fog_sw/releases/download/ver_ssrc_rel_2/ros-foxy-px4-msgs_2.0.1-0focal_amd64.deb -o ros-foxy-px4-msgs.deb \
    && dpkg -i ros-foxy-px4-msgs.deb \
    && rm ros-foxy-px4-msgs.deb

WORKDIR /build

COPY . .

ARG BUILD_NUMBER

RUN cd packaging/ \
    && ./package.sh ${BUILD_NUMBER}

FROM scratch

COPY --from=communication_link_builder /build/*.deb /packages/
