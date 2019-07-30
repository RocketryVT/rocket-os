// watchdog.cpp

#include <ros/ros.h>
#include <std_msgs/Duration.h>
#include <ros/master.h>
#include <boost/asio/ip/host_name.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "watchdog");
    ros::NodeHandle nh("~");

    ros::Time start = ros::Time::now();

    int frequency = nh.param("frequency", 1);
    auto pub = nh.advertise<std_msgs::Duration>("/uptime", 100);
    ros::Rate rate(frequency);

    std::string uri = ros::master::getURI();
    ROS_INFO("Roscore running on URI: %s", uri.c_str());
    ROS_INFO("Hostname is %s", boost::asio::ip::host_name().c_str());

    uint8_t counter = 0;
    while (ros::ok())
    {
        std_msgs::Duration uptime;
        uptime.data = ros::Time::now() - start;
        pub.publish(uptime);

        ROS_INFO_DELAYED_THROTTLE(60, "Runtime has reached %d minutes.",
            static_cast<int>(uptime.data.toSec()/60));
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
