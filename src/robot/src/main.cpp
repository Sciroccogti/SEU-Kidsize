#include <ros/ros.h>
#include <ros/package.h>
#include <robot/robot.hpp>
#include <common/basic_parser.hpp>
#include <std_srvs/Empty.h>
#include <common/BoneLength.h>
#include <common/Kinematics.h>
#include <common/BodyAngles.h>
#include <common/HeadAngles.h>
#include <common/QueueSize.h>
#include <common/VirtualRobot.h>
#include <common/AddAngles.h>
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
std::queue<BodyAngles> bodyAngles;
std::queue<HeadAngles> headAngles;
std::mutex bodyMutex, headMutex;

bool GetVirtual(VirtualRobot::Request &req, VirtualRobot::Response &res);
bool UpdateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool GetBoneLength(BoneLength::Request &req, BoneLength::Response &res);
bool InverseKinematics(Kinematics::Request &req, Kinematics::Response &res);
bool GetQueueSize(QueueSize::Request &req, QueueSize::Response &res);
bool AddAnglesToQue(AddAngles::Request &req, AddAngles::Response &res);

bool UpdateParams();

void UpdateMotor(const ros::TimerEvent& event);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot");
    ros::NodeHandle node;
    
    if(!UpdateParams())
    {
        ROS_ERROR("update params failed");
        return 0;
    }

    ros::ServiceServer pServer = node.advertiseService("/paramservice", UpdateParams);
    std::string robot_file, offset_file;
    bool vr_robot;
    
    try{
        ros::param::get("robot_file", robot_file);
        ros::param::get("offset_file", offset_file);
        ros::param::get("vr_robot", vr_robot);
    }
    catch(ros::InvalidNameException &e){
        ROS_ERROR("%s", e.what());
        return 0;
    }
    maxwell = std::make_shared<Robot>(robot_file, offset_file);
    std::shared_ptr<Mcu> mcu = std::make_shared<Mcu>();
    mcu->open();

    ros::ServiceServer boneSrv = node.advertiseService("/robotbone", GetBoneLength);
    ros::ServiceServer IKSrv = node.advertiseService("/kinematics", InverseKinematics);
    ros::ServiceServer aqSizeSrv = node.advertiseService("/queuesize", GetQueueSize);
    ros::ServiceServer addAglSrv = node.advertiseService("/addangles", AddAnglesToQue);
    Mcu::McuPacket mcuPkt;
    int motor_count = maxwell->joint_count();
    bool dxlConnected=false;
    
    ros::ServiceServer VRSrv;
    if(vr_robot)
    {
        ROS_INFO("virtual robot used");
        VRSrv = node.advertiseService("/virtualrobot", GetVirtual);
    }
    ros::Rate loop_rate(200);
    uint32_t req_cnt=0;
    uint8_t led_status=0;
    while(ros::ok())
    {
        if(mcu->isOpen())
        {
            mcuPkt = mcu->readRequest();
            if(mcuPkt.type == Mcu::REQ_DATA)
            {
                req_cnt++;
                if(req_cnt%50==0)
                {
                    Mcu::McuPacket pkt;
                    pkt.type = Mcu::LED_DATA;
                    pkt.len = 2;
                    pkt.data[0] = led_status;
                    pkt.data[1] = led_status;
                    led_status = 1-led_status;
                    mcu->sendPacket(pkt);
                }
                if(mcuPkt.data[0] == 1)
                {
                    dxlConnected = true;
                    bodyMutex.lock();
                    if(!bodyAngles.empty())
                    {
                        maxwell->set_body(bodyAngles.front());
                        bodyAngles.pop();
                    }
                    bodyMutex.unlock();
                    headMutex.lock();
                    if(!headAngles.empty())
                    {
                        maxwell->set_head(headAngles.front());
                        headAngles.pop();
                    }
                    headMutex.unlock();
                    Mcu::McuPacket pkt;
                    int i=0;
                    pkt.type = Mcu::MOTOR_DATA;
                    pkt.len = motor_count*5;
                    for(auto &jt:maxwell->get_joint_map())
                    {
                        JointPtr &joint = jt.second;
                        float deg = (joint->current_deg+joint->offset)*joint->inverse;
                        uint32_t gpos = float2dxl(deg);
                        pkt.data[i*5+0] = static_cast<uint8_t>(joint->jid);
                        pkt.data[i*5+1] = SEU_LOBYTE(SEU_LOWORD(gpos));
                        pkt.data[i*5+2] = SEU_HIBYTE(SEU_LOWORD(gpos));
                        pkt.data[i*5+3] = SEU_LOBYTE(SEU_HIWORD(gpos));
                        pkt.data[i*5+4] = SEU_HIBYTE(SEU_HIWORD(gpos));
                        i++;
                    }
                    mcu->sendPacket(pkt);
                }
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

bool AddAnglesToQue(AddAngles::Request &req, AddAngles::Response &res)
{
    if(req.part == "body")
    {
        bodyMutex.lock();
        bodyAngles.push(req.body);
        bodyMutex.unlock();
    }
    else if(req.part == "head")
    {
        bodyMutex.lock();
        headAngles.push(req.head);
        bodyMutex.unlock();
    }
}

bool UpdateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    return true;
}

bool GetBoneLength(BoneLength::Request &req, BoneLength::Response &res)
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

bool GetQueueSize(QueueSize::Request &req, QueueSize::Response &res)
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

bool InverseKinematics(Kinematics::Request &req, Kinematics::Response &res)
{
    //ROS_INFO("InverseKinematics: %d", req.part);
    RobotPose bodyPose, endPose;
    bodyPose.x = req.body.translation.x;
    bodyPose.y = req.body.translation.y;
    bodyPose.z = req.body.translation.z;
    bodyPose.roll = req.body.rotation.x;
    bodyPose.pitch = req.body.rotation.y;
    bodyPose.yaw = req.body.rotation.z;

    endPose.x = req.end.translation.x;
    endPose.y = req.end.translation.y;
    endPose.z = req.end.translation.z;
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

bool GetVirtual(VirtualRobot::Request &req, VirtualRobot::Response &res)
{
    bodyMutex.lock();
    if(!bodyAngles.empty())
    {
        maxwell->set_body(bodyAngles.front());
        bodyAngles.pop();
    }
    bodyMutex.unlock();
    headMutex.lock();
    if(!headAngles.empty())
    {
        maxwell->set_head(headAngles.front());
        headAngles.pop();
    }
    headMutex.unlock();
    res.head.yaw = maxwell->get_joint("jhead1")->current_deg;
    res.head.pitch = maxwell->get_joint("jhead2")->current_deg;

    res.body.left_shoulder = maxwell->get_joint("jlshoulder1")->current_deg;
    res.body.left_elbow = maxwell->get_joint("jlelbow")->current_deg;
    res.body.right_shoulder = maxwell->get_joint("jrshoulder1")->current_deg;
    res.body.right_elbow = maxwell->get_joint("jrelbow")->current_deg;

    res.body.left_hip_yaw = maxwell->get_joint("jlhip3")->current_deg;
    res.body.left_hip_roll = maxwell->get_joint("jlhip2")->current_deg;
    res.body.left_hip_pitch = maxwell->get_joint("jlhip1")->current_deg;
    res.body.left_knee = maxwell->get_joint("jlknee")->current_deg;
    res.body.left_ankle_pitch = maxwell->get_joint("jlankle2")->current_deg;
    res.body.left_ankle_roll = maxwell->get_joint("jlankle1")->current_deg;

    res.body.right_hip_yaw = maxwell->get_joint("jrhip3")->current_deg;
    res.body.right_hip_roll = maxwell->get_joint("jrhip2")->current_deg;
    res.body.right_hip_pitch = maxwell->get_joint("jrhip1")->current_deg;
    res.body.right_knee = maxwell->get_joint("jrknee")->current_deg;
    res.body.right_ankle_pitch = maxwell->get_joint("jrankle2")->current_deg;
    res.body.right_ankle_roll = maxwell->get_joint("jrankle1")->current_deg;

    return true;
}

bool UpdateParams()
{
    const std::string root_cfg_file = "config.conf";
    std::string cfgpath = ros::package::getPath("config")+"/conf/";

    string cfgfile(cfgpath+root_cfg_file);
    bpt::ptree pt;
    if(!parse_file(cfgfile, pt)){
        ROS_ERROR("parser cfgfile: %s failed", cfgfile.c_str());
        return false;
    }
    int id = pt.get<int>("id");
    try
    {
        for(auto p:pt)
        {
            if(!p.second.data().empty())
            {
                string data=p.second.data();
                if(p.first.find("file")!=string::npos){
                    data = cfgpath+data;
                }
                if(data == "true")
                {
                    ros::param::set(p.first, true);
                }
                else if(data == "false")
                {
                    ros::param::set(p.first, false);
                }
                else
                {
                    bool ok=false;
                    try{
                        int d = std::stoi(data);
                        ros::param::set(p.first, d);
                        ok = true;
                    }
                    catch(std::invalid_argument&){
                    }
                    if(!ok)
                    {
                        try{
                            float d = std::stof(data);
                            ros::param::set(p.first, d);
                            ok = true;
                        }
                        catch(std::invalid_argument&){
                        }
                    }
                    if(!ok) ros::param::set(p.first, data);
                }
            }
        }
        std::string temp = "players." + std::to_string(id);
        bpt::ptree pr = pt.get_child(temp);
        for (auto p : pr)
        {
            string data=p.second.data();
            if(p.first.find("file")!=string::npos){
                data = cfgpath+data;
            }
            ros::param::set(p.first, data);
        }
    }
    catch (bpt::ptree_error &e)
    {
        ROS_ERROR("%s", e.what());
        return false;
    }
    return true;
}