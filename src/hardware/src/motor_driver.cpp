// motor_driver.cpp

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

const uint8_t STOP = 0, CLOCKWISE = 1, COUNTERCLOCKWISE = 2;

void recieve_motor_command(const std_msgs::UInt8 &mc)
{
    if (mc.data == STOP)
    {
        ROS_INFO("Stopping the motor");
    }
    else if (mc.data == CLOCKWISE)
    {
        ROS_INFO("Turning the motor clockwise");
    }
    else if (mc.data == COUNTERCLOCKWISE)
    {
        ROS_INFO("Turning the motor counterclockwise");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_driver");
    ros::NodeHandle nh("~");

    ROS_DEBUG("Debug message");

    int positive_pin = nh.param("positive", 20);
    int negative_pin = nh.param("negative", 21);
    ROS_INFO("Starting motor driver on pins %d, %d",
        positive_pin, negative_pin);

    auto sub = nh.subscribe("command", 100, recieve_motor_command);

    ros::spin();

    return 0;
}
