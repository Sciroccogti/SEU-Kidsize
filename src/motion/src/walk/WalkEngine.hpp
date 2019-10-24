#ifndef __WALK_ENGINE_HPP
#define __WALK_ENGINE_HPP

#include <map>
#include <thread>
#include <mutex>
#include <atomic>
#include <eigen3/Eigen/Dense>
#include "IKWalk.hpp"
#include <ros/ros.h>

class WalkEngine
{
public:
    WalkEngine(ros::Publisher &pub);
    void runWalk(Eigen::Vector3d p, int steps, double& phase, double& time);
    
private:
    double engine_frequency_;
    double time_length_;
    double XOffset_, YOffset_, DOffset_;
    Rhoban::IKWalkParameters params_;
    ros::Publisher &bodyPublisher_;
    Eigen::Vector2d xrange_, yrange_, drange_;
};

#endif
