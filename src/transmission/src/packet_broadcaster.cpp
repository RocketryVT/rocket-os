// transmission_node.cpp

#include "ros/ros.h"
#include "transmission/Packet.h"

#include <transmission/packet.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "packet_broadcaster");
    ros::NodeHandle nh;

    if (argc < 2)
    {
        ROS_ERROR("Requires a filename to read.");
        return 1;
    }

    std::string filename(argv[1]);

    ROS_INFO("Loading %s", filename.c_str());

    ros::Publisher publisher = nh.advertise<
        transmission::Packet>("recieved_packets", 1000);

    auto packets = rvt::fromFile(filename, 0xAA14);
    ROS_INFO("Loaded %d packets", (int) packets.size());

    size_t packet_index = 0;
    while (ros::ok() && packet_index < packets.size())
    {
        const auto& to_publish = packets[packet_index];
        transmission::Packet msg;
        msg.header.stamp = ros::Time::now();
        msg.data = to_publish.data();
        msg.time = to_publish.time();
        msg.id = to_publish.id();
        msg.sync_bytes = to_publish.sync_bytes();
        msg.checksum = to_publish.checksum();
        publisher.publish(msg);
        ++packet_index;
        ros::spinOnce();
    }

    ros::spin();

    return 0;
}
