
#include "models.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "models_node");
    ros::NodeHandle nh;

    rvt::foo(4);

    ros::spin();

    return 0;
}
