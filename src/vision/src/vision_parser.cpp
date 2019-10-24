#include "vision_parser.hpp"

using namespace common;

bool parse(const std::string &filename, common::CameraProperty &camera_pro)
{
    bpt::ptree pt;

    if (!get_tree_from_file(filename, pt))
    {
        return false;
    }

    try{
        camera_pro.exposure_time = pt.get<double>("exposure_time");
        camera_pro.exposure_gain = pt.get<double>("exposure_gain");
        camera_pro.red_gain = pt.get<double>("red_gain");
        camera_pro.green_gain = pt.get<double>("green_gain");
        camera_pro.blue_gain = pt.get<double>("blue_gain");
        camera_pro.saturation = pt.get<double>("saturation");
        camera_pro.contrast = pt.get<double>("contrast");
        camera_pro.gamma = pt.get<double>("gamma");
    }
    catch (bpt::ptree_error &e)
    {
        return false;
    }
    return true;
}

bool parse(const std::string &filename, CameraParams &param)
{
    bpt::ptree pt;
    if (!get_tree_from_file(filename, pt))
    {
        return false;
    }
    try
    {
        param.fx = pt.get<float>("fx");
        param.fy = pt.get<float>("fy");
        param.cx = pt.get<float>("cx");
        param.cy = pt.get<float>("cy");
        param.k1 = pt.get<float>("k1");
        param.k2 = pt.get<float>("k2");
        param.p1 = pt.get<float>("p1");
        param.p2 = pt.get<float>("p2");
    }
    catch (bpt::ptree_error &e)
    {
        return false;
    }

    return true;
}