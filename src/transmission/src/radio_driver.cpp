// transmission_node.cpp

#include "ros/ros.h"
#include <transmission/Packet.h>
#include "transmission/AckPacket.h"

#include <transmission/packet.hpp>

void send_packet(const transmission::Packet &packet)
{
    // todo
    ROS_INFO("Sending packet...");
}

bool send_with_reply(transmission::AckPacket::Request &request,
                     transmission::AckPacket::Response &response)
{
    // todo
    ROS_INFO("Sending packet and awaiting response...");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radio_driver");
    ros::NodeHandle nh;

    ros::Publisher publisher = nh.advertise<
        transmission::Packet>("incoming", 1000);
    auto sub = nh.subscribe("outgoing", 1000, send_packet);
    auto srv = nh.advertiseService("send_with_reply", send_with_reply);

    ros::spin();

    return 0;
}
