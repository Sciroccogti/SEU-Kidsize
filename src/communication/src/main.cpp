#include <ros/ros.h>
#include "gctrl.hpp"

boost::asio::io_service udp_service;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "communication");
    ros::NodeHandle node;
    bool params_seted = false;
    while (!params_seted && ros::ok())
    {
        try
        {
        ros::param::get("params", params_seted);
        }
        catch (const ros::InvalidNameException &e)
        {
        ROS_WARN("%s", e.what());
        }
        usleep(100000);
    }
    if(!params_seted) return 0;
    GameCtrl gc(node, udp_service);
    gc.start();
    udp_service.run();
    gc.stop();
    return 0;
}