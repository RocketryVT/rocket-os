# rocket-os

## Installing Docker

Install Docker Desktop and follow the tutorial here:

(Windows)   https://hub.docker.com/editions/community/docker-ce-desktop-windows

(Mac)       https://hub.docker.com/editions/community/docker-ce-desktop-mac

## Setup

To build the development environment docker container:
`docker build . -t rvt-devel`

To run the container, which provides an Ubuntu + ROS kinetic environment:
`docker run -itv rvt-devel:/home/rocketry rvt-devel bash`

## Building the Project

To build the rocket firmware:

```
cd ~/rocket-os/
catkin_make
```
