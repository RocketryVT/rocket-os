// transmission_node.cpp

#include "ros/ros.h"
#include <transmission/Packet.h>
#include "transmission/AckPacket.h"
#include <transmission/packet.hpp>
#include <cstdlib>

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

    auto pub = nh.advertise<transmission::Packet>("incoming", 1000);
    auto sub = nh.subscribe("outgoing", 1000, send_packet);
    auto srv = nh.advertiseService("send_with_reply", send_with_reply);

    ros::Rate rate(2);
    while (ros::ok())
    {
        rvt::packet packet;

        auto now = ros::Time::now();
        packet.time() = static_cast<uint64_t>(now.toSec()*1000);
        packet.id() = rand() % 4000;
        for (size_t i = 0; i < rand() % 200; ++i)
            packet.data().push_back(rand() % 256);
        packet.make_valid();

        transmission::Packet msg;
        msg.sync_bytes = 0xAA55;
        msg.id = rand() % 2000;
        msg.time = packet.time();
        msg.data = packet.data();
        msg.checksum = packet.checksum();

        pub.publish(msg);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
