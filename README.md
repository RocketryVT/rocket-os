# rocket-os

## Installing Docker

Install Docker Desktop and follow the tutorial here:

(Windows)   https://hub.docker.com/editions/community/docker-ce-desktop-windows

(Mac)       https://hub.docker.com/editions/community/docker-ce-desktop-mac

## Setup

To build the development environment docker container:
`docker build . -t rvt-devel`

To run the container, which provides an Ubuntu + ROS kinetic environment:
`docker run -it rvt-devel bash`
