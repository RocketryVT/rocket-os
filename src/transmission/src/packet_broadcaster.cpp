// transmission_node.cpp

#include "ros/ros.h"
#include "transmission/Packet.h"

#include <transmission/packet.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "packet_broadcaster");
    ros::NodeHandle nh;

    ros::Publisher publisher = nh.advertise<
        transmission::Packet>("recieved_packets", 1000);

    ros::Rate rate(4);

    while (ros::ok())
    {
        transmission::Packet msg;
        msg.header.stamp = ros::Time::now();
        publisher.publish(msg);

        rate.sleep();
    }

    return 0;
}
