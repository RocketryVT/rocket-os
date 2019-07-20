// motor_driver.cpp

#include <ros/ros.h>
#include <hardware/MotorCommand.h>

void recieve_motor_command(const hardware::MotorCommand &mc)
{
    ROS_INFO("Got a motor command.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_driver");
    ros::NodeHandle nh;

    ROS_INFO("Starting motor driver");

    auto sub = nh.subscribe("motor_cmd", 100, recieve_motor_command);

    ros::spin();
}
