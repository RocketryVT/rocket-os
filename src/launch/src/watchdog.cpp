// watchdog.cpp

#include <ros/ros.h>
#include <std_msgs/Duration.h>
#include <ros/master.h>
#include <boost/asio/ip/host_name.hpp>

ros::Time start;
ros::Publisher pub_uptime;
std::vector<std::string> nodes;

void keep_time(const ros::TimerEvent &event)
{
    std_msgs::Duration uptime;
    uptime.data = event.current_real - start;
    pub_uptime.publish(uptime);
    ROS_INFO_DELAYED_THROTTLE(60, "Runtime has reached %d minutes.",
        static_cast<int>(uptime.data.toSec()/60));
}

void check_on_nodes(const ros::TimerEvent &event)
{
    std::vector<std::string> new_nodes;
    ros::master::getNodes(new_nodes);

    std::sort(begin(new_nodes), end(new_nodes));

    std::vector<std::string> changed;
    std::set_symmetric_difference(begin(nodes), end(nodes),
        begin(new_nodes), end(new_nodes), std::back_inserter(changed));

    for (const auto& e : changed)
    {
        if (std::find(begin(new_nodes), end(new_nodes), e) != end(new_nodes))
        {
            ROS_INFO("New node registered: %s", e.c_str());
        }
        else if (std::find(begin(nodes), end(nodes), e) != end(nodes))
        {
            ROS_INFO("Node killed: %s", e.c_str());
        }
    }

    nodes = new_nodes;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "watchdog");
    ros::NodeHandle nh("~");

    start = ros::Time::now();

    pub_uptime = nh.advertise<std_msgs::Duration>("/uptime", 100);
    ROS_INFO("Roscore running on URI: %s", ros::master::getURI().c_str());
    ROS_INFO("Hostname is %s", boost::asio::ip::host_name().c_str());

    auto keep_timer = nh.createTimer(ros::Duration(1), keep_time);
    auto check_timer = nh.createTimer(ros::Duration(0.1), check_on_nodes);

    ros::spin();

    return 0;
}
