// solenoid_driver.cpp

#include <ros/ros.h>
#include <hardware/SolenoidCommand.h>

void recieve_solenoid_command(const hardware::SolenoidCommand &sc)
{
    ROS_INFO("Got a solenoid command.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "solenoid_driver");
    ros::NodeHandle nh;

    ROS_INFO("Starting solenoid driver");

    auto sub = nh.subscribe("solenoid_cmd", 100, recieve_solenoid_command);

    ros::spin();
}
