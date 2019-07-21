// pressure_transducer_driver.cpp

#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pressure_transducer_driver");
    ros::NodeHandle nh("~");

    int positive_pin, negative_pin;
    if (!nh.getParam("positive", positive_pin) ||
        !nh.getParam("negative", negative_pin))
    {
        ROS_FATAL("Failed to get pressure transducer pin mappings!");
        ros::shutdown();
    }
    ROS_INFO("Starting pressure_transducer driver on pins %d, %d",
        positive_pin, negative_pin);

    auto pub = nh.advertise<std_msgs::Float64>("pressure", 100);

    ros::Rate rate(20); // update pressure at 20 hz
    while (ros::ok())
    {
        std_msgs::Float64 update;
        update.data = 0; // replace with pressure reading
        pub.publish(update);
        rate.sleep();
    }

    return 0;
}
