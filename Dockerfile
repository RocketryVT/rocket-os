FROM ros:kinetic-ros-base

RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -y \
    git doxygen gdb \
    vim nano sudo gcc g++ fortune \
    cowsay tree bsdmainutils \
    bash-completion byobu man ntpdate \
    build-essential python-dev python-pip \
    python-smbus wget

RUN useradd -ms /bin/bash rocketry
RUN usermod -aG sudo rocketry
RUN echo root:rocketry | chpasswd
RUN echo rocketry:rocketry | chpasswd
RUN chown rocketry:rocketry /home/rocketry
RUN echo "rocketry ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

RUN python -m pip install --user rospy_message_converter

USER rocketry
WORKDIR /home/rocketry
RUN touch ~/.hushlogin
