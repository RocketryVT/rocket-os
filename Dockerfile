FROM ros:kinetic-ros-base

RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -y \
    git doxygen gdb \
    vim sudo gcc g++ fortune \
    cowsay tree bsdmainutils \
    bash-completion byobu

RUN useradd -ms /bin/bash rocketry
RUN usermod -aG sudo rocketry
RUN echo root:rocketry | chpasswd
RUN echo rocketry:rocketry | chpasswd
RUN chown rocketry:rocketry /home/rocketry
RUN echo "rocketry ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

USER rocketry
WORKDIR /home/rocketry

RUN git clone https://github.com/RocketryVT/rocket-os ~/rocket-os
