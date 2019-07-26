// pose_estimator.cpp

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

/*
 * === NAV_MSGS::ODOMETRY =======================
 *
 * std_msgs/Header header
 *   uint32 seq
 *   time stamp
 *   string frame_id
 * string child_frame_id
 * geometry_msgs/PoseWithCovariance pose
 *   geometry_msgs/Pose pose
 *     geometry_msgs/Point position
 *       float64 x
 *       float64 y
 *       float64 z
 *     geometry_msgs/Quaternion orientation
 *       float64 x
 *       float64 y
 *       float64 z
 *       float64 w
 *   float64[36] covariance
 * geometry_msgs/TwistWithCovariance twist
 *   geometry_msgs/Twist twist
 *     geometry_msgs/Vector3 linear
 *       float64 x
 *       float64 y
 *       float64 z
 *     geometry_msgs/Vector3 angular
 *       float64 x
 *       float64 y
 *       float64 z
 *   float64[36] covariance
 */

std_msgs::Float64 last_pressure;
geometry_msgs::Quaternion last_orientation;
geometry_msgs::Accel last_acceleration;

void recieve_pressure(const std_msgs::Float64 &msg)
{
    last_pressure = msg;
}

void recieve_orientation(const geometry_msgs::Quaternion &msg)
{
    last_orientation = msg;
}

void recieve_acceleration(const geometry_msgs::Accel &msg)
{
    last_acceleration = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimator");
    ros::NodeHandle nh("~");

    auto sub_pressure =
        nh.subscribe("pressure", 100, recieve_pressure);
    auto sub_orientation =
        nh.subscribe("orientation", 100, recieve_orientation);
    auto sub_acceleration =
        nh.subscribe("acceleration", 100, recieve_acceleration);

    auto pub = nh.advertise<nav_msgs::Odometry>("odometry", 100);

    int frequency = nh.param("frequency", 20);
    ros::Rate rate(frequency);
    while (ros::ok())
    {
        nav_msgs::Odometry odom;
        odom.pose.pose.orientation = last_orientation;
        odom.pose.pose.position.z = last_pressure.data - 101325;
        pub.publish(odom);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
