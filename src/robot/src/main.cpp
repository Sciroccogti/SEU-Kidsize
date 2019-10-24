#include <ros/ros.h>
#include <robot/robot.hpp>
#include <common/basic_parser.hpp>
#include <std_srvs/Empty.h>
#include <common/BoneLength.h>
#include <common/Kinematics.h>
#include <common/BodyAngles.h>
#include <common/HeadAngles.h>
#include <common/QueueSize.h>
#include <common/common.hpp>
#include "dev/imu.hpp"
#include "dev/mcu.hpp"
#include <queue>

using namespace std;
using namespace common;
using namespace robot;
using namespace seumath;
using namespace Eigen;

std::shared_ptr<Robot> maxwell;
std::queue<common::BodyAngles> bodyAngles;
std::queue<common::HeadAngles> headAngles;
std::mutex bodyMutex, headMutex;

bool UpdateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool GetBoneLength(common::BoneLength::Request &req, common::BoneLength::Response &res);
bool InverseKinematics(common::Kinematics::Request &req, common::Kinematics::Response &res);
bool GetQueueSize(common::QueueSize::Request &req, common::QueueSize::Response &res);
void AddBodyAngles(const common::BodyAngles::ConstPtr &p);
void AddHeadAngles(const common::HeadAngles::ConstPtr &p);
void UpdateMotor(const ros::TimerEvent& event);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot");
    ros::NodeHandle node;

    if(argc<3){
        ROS_ERROR("no cfg file set");
        return 0;
    }
    int id = stoi(string(argv[1]));
    string cfgfile(argv[2]);
    common::bpt::ptree pt;
    if(!common::parse_file(cfgfile, pt)){
        ROS_ERROR("parser cfgfile: %s failed", cfgfile.c_str());
        return 0;
    }
    int pos = cfgfile.find_last_of("/");
    string root = cfgfile.substr(0, pos+1);
    try
    {
        for(auto p:pt)
        {
            if(!p.second.data().empty())
            {
                string data=p.second.data();
                if(p.first.find("file")!=string::npos){
                    data = root+data;
                }
                ros::param::set(p.first, data);
            }
        }
        ros::param::set("id", id);
        std::string temp = "players." + std::to_string(id);
        bpt::ptree pr = pt.get_child(temp);
        for (auto p : pr)
        {
            string data=p.second.data();
            if(p.first.find("file")!=string::npos){
                data = root+data;
            }
            ros::param::set(p.first, data);
        }
    }
    catch (bpt::ptree_error &e)
    {
        ROS_ERROR("%s", e.what());
        return 0;
    }
    
    ros::ServiceServer pServer = node.advertiseService("/paramservice", UpdateParams);

    std::string robot_file, offset_file;
    try{
        ros::param::get("robot_file", robot_file);
        ros::param::get("offset_file", offset_file);
    }
    catch(ros::InvalidNameException &e){
        ROS_ERROR("%s", e.what());
        return 0;
    }

    maxwell = std::make_shared<Robot>(robot_file, offset_file);
    std::shared_ptr<Mcu> mcu = std::make_shared<Mcu>();

    ros::ServiceServer boneSrv = node.advertiseService("/robotbone", GetBoneLength);
    ros::ServiceServer IKSrv = node.advertiseService("/kinematics", InverseKinematics);
    ros::ServiceServer aqSizeSrv = node.advertiseService("/queuesize", GetQueueSize);
    ros::Subscriber bAnglesSub = node.subscribe("/bodyangles", 1, AddBodyAngles);
    ros::Subscriber hAnglesSub = node.subscribe("/headangles", 1, AddBodyAngles);
    Mcu::McuPacket mcuPkt;
    while(ros::ok())
    {
        mcuPkt = mcu->readPacket();
        switch(mcuPkt.type)
        {
        case Mcu::REQ_DATA:
            break;
        default:
            break;
        }
        ros::spinOnce();
    }
    return 0;
}


void AddBodyAngles(const common::BodyAngles::ConstPtr &p)
{
    bodyMutex.lock();
    bodyAngles.push(*p);
    bodyMutex.unlock();
}

void AddHeadAngles(const common::HeadAngles::ConstPtr &p)
{
    bodyMutex.lock();
    headAngles.push(*p);
    bodyMutex.unlock();
}

bool UpdateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    return true;
}

bool GetBoneLength(common::BoneLength::Request &req, common::BoneLength::Response &res)
{
    robot::BonePtr bone = maxwell->get_bone(req.name);
    if(!bone.get())
    {
        ROS_ERROR("can not find bone named: %s", req.name.c_str());
        return false;
    }
    res.length = bone->length;
    return true;
}

bool GetQueueSize(common::QueueSize::Request &req, common::QueueSize::Response &res)
{
    if(req.name == "body")
    {
        bodyMutex.lock();
        res.size = bodyAngles.size();
        bodyMutex.unlock();
    }
    else if(req.name == "head")
    {
        headMutex.lock();
        res.size = headAngles.size();
        headMutex.unlock();
    }
    return true;
}

bool InverseKinematics(common::Kinematics::Request &req, common::Kinematics::Response &res)
{
    RobotPose bodyPose, endPose;
    bodyPose.x = req.body.translation.x;
    bodyPose.y = req.body.translation.y;
    bodyPose.z = req.body.translation.y;
    bodyPose.roll = req.body.rotation.x;
    bodyPose.pitch = req.body.rotation.y;
    bodyPose.yaw = req.body.rotation.z;

    endPose.x = req.end.translation.x;
    endPose.y = req.end.translation.y;
    endPose.z = req.end.translation.y;
    endPose.roll = req.end.rotation.x;
    endPose.pitch = req.end.rotation.y;
    endPose.yaw = req.end.rotation.z;

    TransformMatrix bodyMat = maxwell->get_body_mat_from_pose(bodyPose);
    TransformMatrix endMat;
    vector<double> degs;
    bool ok=false;

    if(req.part == req.LEFT_FOOT){
        endMat = maxwell->get_foot_mat_from_pose(endPose, true);
        ok = maxwell->leg_inverse_kinematics(bodyMat, endMat, degs, true);
    }
    else if(req.part == req.RIGHT_FOOT){
        endMat = maxwell->get_foot_mat_from_pose(endPose, false);
        ok = maxwell->leg_inverse_kinematics(bodyMat, endMat, degs, false);
    }
    else if(req.part == req.LEFT_HAND || req.part == req.RIGHT_HAND){
        Vector3d hand;
        hand.x() = endPose.x;
        hand.y() = endPose.y;
        hand.z() = endPose.z;
        ok = maxwell->arm_inverse_kinematics(hand, degs);
    }
    res.success = ok;
    for(int i=0; i<degs.size(); i++)
        res.degs[i] = degs[i];
    return true;
}

void UpdateMotor(const ros::TimerEvent& event)
{
    headMutex.lock();
    if(!headAngles.empty())
    {
        maxwell->set_head(headAngles.front());
        headAngles.pop();
    }
    headMutex.unlock();
    bodyMutex.lock();
    if(!bodyAngles.empty())
    {
        maxwell->set_body(bodyAngles.front());
        bodyAngles.pop();
    }
    bodyMutex.unlock();
}