#include "ActionEngine.hpp"
#include <ros/ros.h>
#include <common/Kinematics.h>
#include <common/AddAngles.h>

using namespace robot;
using namespace seumath;

static void PoseToTrans(const RobotPose &pose, geometry_msgs::Transform &trans)
{
    trans.translation.x = pose.x;
    trans.translation.y = pose.y;
    trans.translation.z = pose.z;
    trans.rotation.x = pose.roll;
    trans.rotation.y = pose.pitch;
    trans.rotation.z = pose.yaw;
}

ActionEngine::ActionEngine()
{
    ros::service::waitForService("/paramservice");
    std::string act_file;
    try{
        ros::param::get("action_file", act_file);
    }
    catch(ros::InvalidNameException &e){
        ROS_ERROR("%s", e.what());
        return;
    }
    ROS_INFO("action_file: %s", act_file.c_str());
    parse(act_file, act_map_, pos_map_);
}

bool ActionEngine::runAction(std::string act)
{
    auto iter=act_map_.find(act);
    if(iter == act_map_.end())
    {
        ROS_ERROR("can not find act: %s", act.c_str());
        return false;
    }
    common::AddAngles addSrv;
    addSrv.request.part = "body";
    common::BodyAngles &bAngles = addSrv.request.body;
    std::vector<PoseMap> poses_temp;
    std::vector<int> pos_times_temp;
    std::vector<robot::RobotOnePos> poses=iter->second.poses;
    for(int i=0; i<poses.size(); i++)
    {
        pos_times_temp.push_back(poses[i].act_time);
        poses_temp.push_back(pos_map_[poses[i].pos_name].pose_info);
    }
    if(poses_temp.empty()) return false;
    int act_time;
    std::map<int, float> one_pos_deg;
    std::map<std::string, float> jdegs;
    std::map<robot::RobotMotion, robot::RobotPose> pos1, pos2;
    act_time = pos_times_temp[0];
    pos1 = poses_temp[0];

    bool ret = get_degs(pos1, bAngles);
    if(!ret) return false;
    ros::service::call("/addangles", addSrv);
    if(poses_temp.size()<2) return true;
    for (int i = 1; i < poses_temp.size(); i++)
    {
        pos1 = poses_temp[i - 1];
        pos2 = poses_temp[i];
        std::vector< std::map<robot::RobotMotion, robot::RobotPose> > act_poses;
        act_poses = get_poses(pos1, pos2, pos_times_temp[i]);
        if(i==poses_temp.size()-1)
            act_poses.push_back(pos2);

        for (auto act_pose : act_poses)
        {
            ret = get_degs(pos1, bAngles);
            if(!ret) return false;
            ros::service::call("/addangles", addSrv);
        }
    }
    return true;
}

bool ActionEngine::get_degs(robot::PoseMap &act_pose,  common::BodyAngles &bAngles)
{
    common::Kinematics ik;
    PoseToTrans(act_pose[MOTION_BODY], ik.request.body);
    PoseToTrans(act_pose[MOTION_LEFT_FOOT], ik.request.end);
    ik.request.part = ik.request.LEFT_FOOT;
    ros::service::call("/kinematics", ik);
    if(!ik.response.success) 
    {
        ROS_WARN("left foot kinematics failed");
        return false;
    }
    bAngles.left_hip_yaw = rad2deg(ik.response.degs[0]);
    bAngles.left_hip_roll = rad2deg(ik.response.degs[1]);
    bAngles.left_hip_pitch = rad2deg(ik.response.degs[2]);
    bAngles.left_knee = rad2deg(ik.response.degs[3]);
    bAngles.left_ankle_pitch = rad2deg(ik.response.degs[4]);
    bAngles.left_ankle_roll = rad2deg(ik.response.degs[5]);

    PoseToTrans(act_pose[MOTION_RIGHT_FOOT], ik.request.end);
    ik.request.part = ik.request.RIGHT_FOOT;
    ros::service::call("/kinematics", ik);
    if(!ik.response.success) 
    {
        ROS_WARN("right foot kinematics failed");
        return false;
    }
    bAngles.right_hip_yaw = rad2deg(ik.response.degs[0]);
    bAngles.right_hip_roll = rad2deg(ik.response.degs[1]);
    bAngles.right_hip_pitch = rad2deg(ik.response.degs[2]);
    bAngles.right_knee = rad2deg(ik.response.degs[3]);
    bAngles.right_ankle_pitch = rad2deg(ik.response.degs[4]);
    bAngles.right_ankle_roll = rad2deg(ik.response.degs[5]);

    PoseToTrans(act_pose[MOTION_LEFT_HAND], ik.request.end);
    ik.request.part = ik.request.LEFT_HAND;
    ros::service::call("/kinematics", ik);
    if(!ik.response.success)
    {
        ROS_WARN("left hand kinematics failed");
        return false;
    }
    bAngles.left_shoulder = rad2deg(ik.response.degs[0]);
    bAngles.left_elbow = rad2deg(ik.response.degs[1]);
    PoseToTrans(act_pose[MOTION_RIGHT_HAND], ik.request.end);
    ik.request.part = ik.request.RIGHT_HAND;
    ros::service::call("/kinematics", ik);
    if(!ik.response.success) 
    {
        ROS_WARN("right hand kinematics failed");
        return false;
    }
    bAngles.right_shoulder = rad2deg(ik.response.degs[0]);
    bAngles.right_elbow = rad2deg(ik.response.degs[1]);
    return true;
}

std::vector< std::map<robot::RobotMotion, robot::RobotPose> > 
    ActionEngine::get_poses(std::map<robot::RobotMotion, robot::RobotPose> &pos1,
            std::map<robot::RobotMotion, robot::RobotPose> &pos2, int act_time)
{
    Eigen::Vector3d poseb1(pos1[robot::MOTION_BODY].x, pos1[robot::MOTION_BODY].y, pos1[robot::MOTION_BODY].z);
    Eigen::Vector3d poseb2(pos2[robot::MOTION_BODY].x, pos2[robot::MOTION_BODY].y, pos2[robot::MOTION_BODY].z);
    Eigen::Vector3d posel1(pos1[robot::MOTION_LEFT_FOOT].x, pos1[robot::MOTION_LEFT_FOOT].y, pos1[robot::MOTION_LEFT_FOOT].z);
    Eigen::Vector3d posel2(pos2[robot::MOTION_LEFT_FOOT].x, pos2[robot::MOTION_LEFT_FOOT].y, pos2[robot::MOTION_LEFT_FOOT].z);
    Eigen::Vector3d poser1(pos1[robot::MOTION_RIGHT_FOOT].x, pos1[robot::MOTION_RIGHT_FOOT].y, pos1[robot::MOTION_RIGHT_FOOT].z);
    Eigen::Vector3d poser2(pos2[robot::MOTION_RIGHT_FOOT].x, pos2[robot::MOTION_RIGHT_FOOT].y, pos2[robot::MOTION_RIGHT_FOOT].z);
    Eigen::Vector3d pposeb1(pos1[robot::MOTION_BODY].pitch, pos1[robot::MOTION_BODY].roll, pos1[robot::MOTION_BODY].yaw);
    Eigen::Vector3d pposeb2(pos2[robot::MOTION_BODY].pitch, pos2[robot::MOTION_BODY].roll, pos2[robot::MOTION_BODY].yaw);
    Eigen::Vector3d pposel1(pos1[robot::MOTION_LEFT_FOOT].pitch, pos1[robot::MOTION_LEFT_FOOT].roll, pos1[robot::MOTION_LEFT_FOOT].yaw);
    Eigen::Vector3d pposel2(pos2[robot::MOTION_LEFT_FOOT].pitch, pos2[robot::MOTION_LEFT_FOOT].roll, pos2[robot::MOTION_LEFT_FOOT].yaw);
    Eigen::Vector3d pposer1(pos1[robot::MOTION_RIGHT_FOOT].pitch, pos1[robot::MOTION_RIGHT_FOOT].roll, pos1[robot::MOTION_RIGHT_FOOT].yaw);
    Eigen::Vector3d pposer2(pos2[robot::MOTION_RIGHT_FOOT].pitch, pos2[robot::MOTION_RIGHT_FOOT].roll, pos2[robot::MOTION_RIGHT_FOOT].yaw);

    Eigen::Vector3d poselh1(pos1[robot::MOTION_LEFT_HAND].x, pos1[robot::MOTION_LEFT_HAND].y, pos1[robot::MOTION_LEFT_HAND].z);
    Eigen::Vector3d poselh2(pos2[robot::MOTION_LEFT_HAND].x, pos2[robot::MOTION_LEFT_HAND].y, pos2[robot::MOTION_LEFT_HAND].z);
    Eigen::Vector3d poserh1(pos1[robot::MOTION_RIGHT_HAND].x, pos1[robot::MOTION_RIGHT_HAND].y, pos1[robot::MOTION_RIGHT_HAND].z);
    Eigen::Vector3d poserh2(pos2[robot::MOTION_RIGHT_HAND].x, pos2[robot::MOTION_RIGHT_HAND].y, pos2[robot::MOTION_RIGHT_HAND].z);

    Eigen::Vector3d dposeb = poseb2 - poseb1;
    Eigen::Vector3d dposel = posel2 - posel1;
    Eigen::Vector3d dposer = poser2 - poser1;
    Eigen::Vector3d dposelh = poselh2 - poselh1;
    Eigen::Vector3d dposerh = poserh2 - poserh1;

    Eigen::Vector3d dpposeb = pposeb2 - pposeb1;
    Eigen::Vector3d dpposel = pposel2 - pposel1;
    Eigen::Vector3d dpposer = pposer2 - pposer1;
    int count;
    count = act_time;
    double dbx = dposeb.x() / count, dby = dposeb.y() / count, dbz = dposeb.z() / count;
    double dbpi = dpposeb.x() / count, dbro = dpposeb.y() / count, dbya = dpposeb.z() / count;
    double dlx = dposel.x() / count, dly = dposel.y() / count, dlz = dposel.z() / count;
    double dlpi = dpposel.x() / count, dlro = dpposel.y() / count, dlya = dpposel.z() / count;
    double drx = dposer.x() / count, dry = dposer.y() / count, drz = dposer.z() / count;
    double drpi = dpposer.x() / count, drro = dpposer.y() / count, drya = dpposer.z() / count;

    double dlhx = dposelh.x() / count, dlhz = dposelh.z() / count;
    double drhx = dposerh.x() / count, drhz = dposerh.z() / count;
    std::vector< std::map<robot::RobotMotion, robot::RobotPose> > act_poses;
    for (int i = 0; i < count; i++)
    {
        std::map<robot::RobotMotion, robot::RobotPose> temp_pose_map;
        robot::RobotPose temp_pose;
        temp_pose.x = poselh1.x() + i * dlhx;
        temp_pose.z = poselh1.z() + i * dlhz;
        temp_pose_map[robot::MOTION_LEFT_HAND] = temp_pose;
        temp_pose.x = poserh1.x() + i * drhx;
        temp_pose.z = poserh1.z() + i * drhz;
        temp_pose_map[robot::MOTION_RIGHT_HAND] = temp_pose;
        temp_pose.x = poseb1.x() + i * dbx;
        temp_pose.y = poseb1.y() + i * dby;
        temp_pose.z = poseb1.z() + i * dbz;
        temp_pose.pitch = pposeb1.x() + i * dbpi;
        temp_pose.roll = pposeb1.y() + i * dbro;
        temp_pose.yaw = pposeb1.z() + i * dbya;
        temp_pose_map[robot::MOTION_BODY] = temp_pose;
        temp_pose.x = posel1.x() + i * dlx;
        temp_pose.y = posel1.y() + i * dly;
        temp_pose.z = posel1.z() + i * dlz;
        temp_pose.pitch = pposel1.x() + i * dlpi;
        temp_pose.roll = pposel1.y() + i * dlro;
        temp_pose.yaw = pposel1.z() + i * dlya;
        temp_pose_map[robot::MOTION_LEFT_FOOT] = temp_pose;
        temp_pose.x = poser1.x() + i * drx;
        temp_pose.y = poser1.y() + i * dry;
        temp_pose.z = poser1.z() + i * drz;
        temp_pose.pitch = pposer1.x() + i * drpi;
        temp_pose.roll = pposer1.y() + i * drro;
        temp_pose.yaw = pposer1.z() + i * drya;
        temp_pose_map[robot::MOTION_RIGHT_FOOT] = temp_pose;
        act_poses.push_back(temp_pose_map);
    }
    return act_poses;
}