// solenoid_driver.cpp

#include <ros/ros.h>
#include <std_msgs/UInt8.h>

const uint8_t CLOSE = 0, OPEN = 1;

void recieve_solenoid_command(const std_msgs::UInt8 &sc)
{
    if (sc.data == CLOSE)
    {
        ROS_INFO("Closing the solenoid!");
    }
    else if (sc.data == OPEN)
    {
        ROS_INFO("Opening the solenoid!");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "solenoid_driver");
    ros::NodeHandle nh("~");

    int positive_pin = nh.param("positive", 20);
    int negative_pin = nh.param("negative", 21);
    ROS_INFO("Starting solenoid driver on pins %d, %d",
        positive_pin, negative_pin);

    auto sub = nh.subscribe("command", 100,
        recieve_solenoid_command);

    ros::spin();
}
