// float_switch_driver.cpp

#include <ros/ros.h>
#include <std_msgs/Bool.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "float_switch_driver");
    ros::NodeHandle nh("~");

    int positive_pin, negative_pin;
    if (!nh.getParam("positive", positive_pin) ||
        !nh.getParam("negative", negative_pin))
    {
        ROS_FATAL("Failed to get thermocouple pin mappings!");
        ros::shutdown();
    }
    ROS_INFO("Starting float switch driver on pins %d, %d",
        positive_pin, negative_pin);

    auto pub = nh.advertise<std_msgs::Bool>("state", 100);

    ros::Rate rate(20); // update float switch at 20 hz
    while (ros::ok())
    {
        std_msgs::Bool update;
        update.data = false; // replace with temperature reading
        pub.publish(update);
        rate.sleep();
    }

    return 0;
}
