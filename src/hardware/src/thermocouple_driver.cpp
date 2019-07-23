// thermocouple_driver.cpp

#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thermocouple_driver");
    ros::NodeHandle nh("~");

    int positive_pin = nh.param("positive", 20);
    int negative_pin = nh.param("negative", 21);
    int frequency = nh.param("frequency", 20);
    ROS_INFO("Starting thermocouple driver on pins %d, %d, "
             "at %d hz", positive_pin, negative_pin, frequency);

    auto pub = nh.advertise<std_msgs::Float64>("temperature", 100);

    ros::Rate rate(frequency);
    while (ros::ok())
    {
        std_msgs::Float64 update;
        update.data = 0; // replace with temperature reading
        pub.publish(update);
        rate.sleep();
    }

    return 0;
}
