// barometer_driver.cpp

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "barometer_driver");
    ros::NodeHandle nh("~");

    int positive_pin = nh.param("positive", 20);
    int negative_pin = nh.param("negative", 21);
    int frequency = nh.param("frequency", 20);
    ROS_INFO("Starting barometer driver on pins %d, %d, "
             "at %d hz", positive_pin, negative_pin, frequency);

    auto pub = nh.advertise<std_msgs::Float64>("pressure", 100);

    ros::Rate rate(frequency);
    while (ros::ok())
    {
        std_msgs::Float64 pressure;
        pressure.data = std::sin(ros::Time::now().toSec())*10 + 101325;
        pub.publish(pressure);
        rate.sleep();
    }

    return 0;
}
