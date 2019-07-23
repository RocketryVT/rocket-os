// imu_driver.cpp

#include <ros/ros.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Quaternion.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_driver");
    ros::NodeHandle nh("~");

    int positive_pin = nh.param("positive", 20);
    int negative_pin = nh.param("negative", 21);
    int frequency = nh.param("frequency", 20);
    ROS_INFO("Starting imu driver on pins %d, %d, "
             "at %d hz", positive_pin, negative_pin, frequency);

    auto accel_pub = nh.advertise<geometry_msgs::Accel>("acceleration", 100);
    auto quat_pub = nh.advertise<geometry_msgs::Quaternion>("orientation", 100);

    ros::Rate rate(frequency);
    while (ros::ok())
    {
        geometry_msgs::Accel accel;
        accel.linear.x = std::sin(ros::Time::now().toSec());
        accel.linear.y = std::sin(ros::Time::now().toSec() + 1);
        accel.linear.z = std::sin(ros::Time::now().toSec()*4 + 1);
        accel.angular.x = std::sin(ros::Time::now().toSec()*2);
        accel.angular.y = std::sin(ros::Time::now().toSec()/1.2 + 3);
        accel.angular.z = std::sin(ros::Time::now().toSec()/3 + 0.2);
        accel_pub.publish(accel);
        geometry_msgs::Quaternion quat;
        quat.w = std::sin(ros::Time::now().toSec()*2);
        quat.x = std::sin(ros::Time::now().toSec());
        quat.y = std::sin(ros::Time::now().toSec() + 1);
        quat.z = std::sin(ros::Time::now().toSec()*4 + 1);
        quat_pub.publish(quat);
        rate.sleep();
    }

    return 0;
}
