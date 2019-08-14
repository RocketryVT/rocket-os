// bare_bones_node.cpp

#include <ros/ros.h>

int main(int argc, char **argv)
{
    // initialize the node
    ros::init(argc, argv, "bare_bones_cpp");

    // do things here

    // pass control back to ROS -
    // this handles events and callbacks which allow
    // other nodes to communicate with each other
    ros::spin();

    return 0;
}
