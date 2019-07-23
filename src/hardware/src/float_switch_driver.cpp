// float_switch_driver.cpp

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "float_switch_driver");
    ros::NodeHandle nh("~");

    int positive_pin = nh.param("positive", 20);
    int negative_pin = nh.param("negative", 21);
    int frequency = nh.param("frequency", 20);
    ROS_INFO("Starting float switch driver on pins %d, %d, "
             "at %d hz", positive_pin, negative_pin, frequency);

    auto pub = nh.advertise<std_msgs::Bool>("state", 100);

    ros::Rate rate(frequency);
    while (ros::ok())
    {
        std_msgs::Bool update;
        update.data = rand() % 2; // replace with float switch reading
        pub.publish(update);
        rate.sleep();
    }

    return 0;
}
