// motor_driver.cpp

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <hardware/MotorCommand.h>

void recieve_motor_command(const hardware::MotorCommand &mc)
{
    ROS_INFO("Got a motor command.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_driver");
    ros::NodeHandle nh("~");

    int positive_pin, negative_pin;
    if (!nh.getParam("positive", positive_pin) ||
        !nh.getParam("negative", negative_pin))
    {
        ROS_FATAL("Failed to get motor pin mappings!");
        ros::shutdown();
    }
    ROS_INFO("Starting motor driver on pins %d, %d",
        positive_pin, negative_pin);

    auto sub = nh.subscribe("motor_cmd", 100, recieve_motor_command);

    ros::spin();

    return 0;
}
