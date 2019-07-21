// solenoid_driver.cpp

#include <ros/ros.h>
#include <hardware/SolenoidCommand.h>

void recieve_solenoid_command(const hardware::SolenoidCommand &sc)
{
    if (sc.command == hardware::SolenoidCommand::CLOSE)
    {
        ROS_INFO("Closing the solenoid!");
    }
    else if (sc.command == hardware::SolenoidCommand::OPEN)
    {
        ROS_INFO("Opening the solenoid!");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "solenoid_driver");
    ros::NodeHandle nh("~");

    int positive_pin, negative_pin;
    if (!nh.getParam("positive", positive_pin) ||
        !nh.getParam("negative", negative_pin))
    {
        ROS_FATAL("Failed to get solenoid pin mappings!");
        ros::shutdown();
    }
    ROS_INFO("Starting solenoid driver on pins %d, %d",
        positive_pin, negative_pin);

    auto sub = nh.subscribe("solenoid_cmd", 100,
        recieve_solenoid_command);

    ros::spin();
}
