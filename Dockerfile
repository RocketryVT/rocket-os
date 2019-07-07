FROM ros:kinetic-ros-base

RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -y \
    git doxygen gdb \
    vim nano sudo gcc g++ fortune \
    cowsay tree bsdmainutils \
    bash-completion byobu man

RUN useradd -ms /bin/bash rocketry
RUN usermod -aG sudo rocketry
RUN echo root:rocketry | chpasswd
RUN echo rocketry:rocketry | chpasswd
RUN chown rocketry:rocketry /home/rocketry
RUN echo "rocketry ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

USER rocketry
WORKDIR /home/rocketry
RUN touch ~/.hushlogin

RUN git clone https://github.com/RocketryVT/rocket-os ~/rocket-os
RUN cp ~/rocket-os/.vimrc ~
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash && \
    cd rocket-os/ && catkin_make || \
    echo -e "\n========== BUILD FAILURE ==========\n"'
RUN echo 'source ~/rocket-os/devel/setup.bash' >> .bashrc
