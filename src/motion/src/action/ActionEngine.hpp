#ifndef __ACTION_ENGINE_HPP
#define __ACTION_ENGINE_HPP

#include <ros/ros.h>
#include <robot/robot.hpp>
#include <seumath/math.hpp>
#include <common/BodyAngles.h>

class ActionEngine
{
public:
    ActionEngine();
    bool runAction(std::string act);

private:
    std::vector< std::map<robot::RobotMotion, robot::RobotPose> > 
            get_poses(std::map<robot::RobotMotion, robot::RobotPose> &pos1,
            std::map<robot::RobotMotion, robot::RobotPose> &pos2, int act_time);

    bool get_degs(robot::PoseMap &act_pose, common::BodyAngles &bAngles);

private:
    robot::ActMap act_map_;
    robot::PosMap pos_map_;
};

#endif
