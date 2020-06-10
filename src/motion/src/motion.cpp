#include "scan/ScanEngine.hpp"
#include "walk/WalkEngine.hpp"
#include <common/BodyAngles.h>
#include <common/HeadAngles.h>
#include <common/BodyTask.h>
#include <common/HeadTask.h>
#include <common/ImuData.h>
#include <common/GetAngles.h>
#include <common/AddAngles.h>
#include <common/GetActions.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <robot/action_engine.hpp>
#include <robot/robot.hpp>
#include <bits/stdc++.h>
#include <ros/ros.h>

using namespace Eigen;


void UpdateBodyTask(const common::BodyTask::ConstPtr &p);
common::BodyTask GetBodyTask();
void UpdateHeadTask(const common::HeadTask::ConstPtr &p);
common::HeadTask GetHeadTask();
void UpdateImu(const common::ImuData::ConstPtr &p);
common::ImuData GetImuData();
bool GetAnglesService(common::GetAngles::Request &req, common::GetAngles::Response &res);
bool AddAnglesService(common::AddAngles::Request &req, common::AddAngles::Response &res);
bool GetActionsService(common::GetActions::Request &req, common::GetActions::Response &res);

std::shared_ptr<robot::ActionEngine> actionEng;
std::shared_ptr<WalkEngine> walkEng;
std::shared_ptr<ScanEngine> scanEng;
std::shared_ptr<robot::Robot> maxwell;

common::BodyTask bodyTask;
std::mutex bodyMutex;
common::HeadTask headTask;
std::mutex headMutex;
common::ImuData imuData;
std::mutex imuMutex;
bool debug = false;
std::deque<common::BodyAngles> bodyAngles;
std::deque<common::HeadAngles> headAngles;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion");
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
  std::string act_file, robot_file, offset_file, walk_file;
  try
  {
    ros::param::get("action_file", act_file);
    ros::param::get("robot_file", robot_file);
    ros::param::get("offset_file", offset_file);
    ros::param::get("walk_file", walk_file);
  }
  catch (ros::InvalidNameException &e)
  {
    ROS_ERROR("%s", e.what());
    return 0;
  }
  maxwell = std::make_shared<robot::Robot>(robot_file, offset_file);
  actionEng = std::make_shared<robot::ActionEngine>(act_file, maxwell);
  walkEng = std::make_shared<WalkEngine>(walk_file, maxwell);
  scanEng = std::make_shared<ScanEngine>();

  ros::Subscriber bodySub = node.subscribe("/task/body", 1, UpdateBodyTask);
  ros::Subscriber headSub = node.subscribe("/task/head", 1, UpdateHeadTask);
  ros::Subscriber imuSub = node.subscribe("/sensor/imu", 1, UpdateImu);
  ros::ServiceServer getAglSrv = node.advertiseService("/getangles", GetAnglesService);
  ros::ServiceServer addAglSrv = node.advertiseService("/addangles", AddAnglesService);
  ros::ServiceServer getActsSrv = node.advertiseService("/getactions", GetActionsService);

  double phase = 0.0;
  bool isWalking = false;
  ros::Rate loop_rate(100);
  std::vector<common::BodyAngles> angles = actionEng->runAction("ready");
  bodyAngles.insert(bodyAngles.end(), angles.begin(), angles.end());
  while (ros::ok())
  {
    if(debug) {
      ros::spinOnce();
      continue;
    }
    if (headAngles.size() < 5)
    {
      auto task = GetHeadTask();
      common::HeadAngles ha;
      ha.pitch = task.pitch;
      ha.yaw = task.yaw;
      headAngles.push_back(ha);
    }
    if (bodyAngles.size() < 5)
    {
      auto task = GetBodyTask();
      auto imu = GetImuData();
      angles.clear();
      if (imu.fall == imu.FALL_FORWARD)
      {
        angles = actionEng->runAction("front_getup");
      }
      else if (imu.fall == imu.FALL_BACKWARD)
      {
        angles = actionEng->runAction("back_getup");
      }
      else if (imu.fall == imu.FALL_LEFT)
      {
        angles = actionEng->runAction("left_getup");
      }
      else if (imu.fall == imu.FALL_RIGHT)
      {
        angles = actionEng->runAction("right_getup");
      }
      else
      {
        if (task.type == common::BodyTask::TASK_WALK)
        {
          if (task.count > 0)
          {
            if (!isWalking)
            {
              angles = walkEng->runWalk(Vector3d(0.0, 0.0, 0.0), 2, phase);
              isWalking = true;
            }
            else
            {
              angles = walkEng->runWalk(Vector3d(task.step, task.lateral, task.turn), task.count, phase);
            }
          }
          else
          {
            if (isWalking)
            {
              angles = walkEng->runWalk(Vector3d(0.0, 0.0, 0.0), 0, phase);
              isWalking = false;
            }
          }
        }
        else if (task.type == common::BodyTask::TASK_ACT)
        {
          if (task.count >= 0)
          {
            if (isWalking)
            {
              angles = walkEng->runWalk(Vector3d(0.0, 0.0, 0.0), 0, phase);
              isWalking = false;
            }
            angles = actionEng->runAction(task.actname);
          }
        }
        else
        {
          if (isWalking)
          {
            angles = walkEng->runWalk(Vector3d(0.0, 0.0, 0.0), 0, phase);
            isWalking = false;
          }
        }
      }
      bodyAngles.insert(bodyAngles.end(), angles.begin(), angles.end());
    }
    ros::spinOnce();
  }
  return 0;
}

void UpdateBodyTask(const common::BodyTask::ConstPtr &p)
{
  std::lock_guard<std::mutex> lk(bodyMutex);
  bodyTask = *p;
}

common::BodyTask GetBodyTask()
{
  std::lock_guard<std::mutex> lk(bodyMutex);
  auto ret = bodyTask;
  bodyTask.count = -1;
  return ret;
}

void UpdateHeadTask(const common::HeadTask::ConstPtr &p)
{
  std::lock_guard<std::mutex> lk(headMutex);
  headTask = *p;
}

common::HeadTask GetHeadTask()
{
  std::lock_guard<std::mutex> lk(headMutex);
  return headTask;
}

void UpdateImu(const common::ImuData::ConstPtr &p)
{
  std::lock_guard<std::mutex> lk(imuMutex);
  imuData = *p;
}

common::ImuData GetImuData()
{
  std::lock_guard<std::mutex> lk(imuMutex);
  return imuData;
}

bool GetAnglesService(common::GetAngles::Request &req, common::GetAngles::Response &res)
{
  //ROS_INFO("getangles");
  bodyMutex.lock();
  if (!bodyAngles.empty())
  {
    maxwell->set_body(bodyAngles.front());
    bodyAngles.pop_front();
  }
  bodyMutex.unlock();
  headMutex.lock();
  if (!headAngles.empty())
  {
    maxwell->set_head(headAngles.front());
    headAngles.pop_front();
  }
  headMutex.unlock();
  if(req.player == "real")
  {
    int sid = maxwell->joint_start_id();
    res.start_id = sid;
    for(int i=0; i<maxwell->joint_count(); i++)
    {
      auto joint = maxwell->get_joint(sid+i);
      float deg = (joint->current_deg + joint->offset) * joint->inverse;
      res.degs.push_back(deg);
    }
  }
  else
  {
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
  }
  return true;
}

bool AddAnglesService(common::AddAngles::Request &req, common::AddAngles::Response &res)
{
    bodyAngles.insert(bodyAngles.end(), req.angles.begin(), req.angles.end());
    debug = true;
    return true;
}

bool GetActionsService(common::GetActions::Request &req, common::GetActions::Response &res)
{
  res.actions = actionEng->getActions();
  return true;
}
