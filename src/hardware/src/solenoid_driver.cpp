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

    int positive_pin = nh.param("positive", 20);
    int negative_pin = nh.param("negative", 21);
    ROS_INFO("Starting solenoid driver on pins %d, %d",
        positive_pin, negative_pin);

    auto sub = nh.subscribe("command", 100,
        recieve_solenoid_command);

    ros::spin();
}
