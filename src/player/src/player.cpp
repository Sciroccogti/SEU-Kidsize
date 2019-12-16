#include "dev/mcu.hpp"
#include <common/AddAngles.h>
#include <common/BodyAngles.h>
#include <common/BoneLength.h>
#include <common/GetAngles.h>
#include <common/HeadAngles.h>
#include <common/ImuData.h>
#include <common/Kinematics.h>
#include <common/QueueSize.h>
#include <common/common.hpp>
#include <queue>
#include <robot/robot.hpp>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

using namespace std;
using namespace common;
using namespace robot;
using namespace seumath;
using namespace Eigen;

bool RobotService(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res);
bool GetBoneLengthService(BoneLength::Request &req, BoneLength::Response &res);
bool InverseKinematicsService(Kinematics::Request &req,
                              Kinematics::Response &res);
bool GetQueueSizeService(QueueSize::Request &req, QueueSize::Response &res);
bool AddAnglesService(AddAngles::Request &req, AddAngles::Response &res);
bool GetAnglesService(GetAngles::Request &req, GetAngles::Response &res);

void SendJointToMCU();

std::shared_ptr<Robot> maxwell;
std::shared_ptr<Mcu> mcu;
std::queue<BodyAngles> bodyAngles;
std::queue<HeadAngles> headAngles;
std::mutex bodyMutex, headMutex;
float imu_init_yaw = 0.0;
void NormalizeIMU(ImuData &idata);
bool dxl_connected = false;
uint32_t start_clock = 0;

int main(int argc, char **argv) {
  ros::init(argc, argv, "maxwell");
  ros::NodeHandle node;
  bool params_seted = false;
  while (!params_seted && ros::ok()) {
    try {
      ros::param::get("params", params_seted);
    } catch (const ros::InvalidNameException &e) {
      ROS_WARN("%s", e.what());
    }
    usleep(100000);
  }
  std::string robot_file, offset_file;
  try {
    ros::param::get("robot_file", robot_file);
    ros::param::get("offset_file", offset_file);
  } catch (ros::InvalidNameException &e) {
    ROS_ERROR("%s", e.what());
    return 0;
  }
  maxwell = std::make_shared<Robot>(robot_file, offset_file);
  mcu = std::make_shared<Mcu>();
  mcu->open();
  start_clock = get_clock();
  ros::ServiceServer rbtSrv = node.advertiseService("/maxwell", RobotService);
  ros::ServiceServer boneSrv =
      node.advertiseService("/bonelength", GetBoneLengthService);
  ros::ServiceServer IKSrv =
      node.advertiseService("/kinematics", InverseKinematicsService);
  ros::ServiceServer aqSizeSrv =
      node.advertiseService("/queuesize", GetQueueSizeService);
  ros::ServiceServer addAglSrv =
      node.advertiseService("/addangles", AddAnglesService);
  ros::ServiceServer getAglSrv =
      node.advertiseService("/getangles", GetAnglesService);

  ros::Publisher imuPublisher = node.advertise<ImuData>("/sensor/imu", 1);
  ros::Publisher headPublisher = node.advertise<HeadAngles>("/sensor/head", 1);
  Mcu::McuPacket mcuPkt;
  ImuData imudata;

  ros::Rate loop_rate(200);
  uint32_t req_cnt = 0;
  uint8_t led_status = 0;
  while (ros::ok()) {
    if (mcu->isOpen()) {
      mcuPkt.type = 0;
      mcuPkt.len = 0;
      bool ok = mcu->readPacket(mcuPkt);
      if (mcuPkt.type == Mcu::REQ_DATA) {
        req_cnt++;
        imudata.stamp = get_clock();
        imudata.pitch = ((float)(int16_t)(mcuPkt.data[Mcu::REQ_DATA_IMU_OFFSET] +
                            (mcuPkt.data[Mcu::REQ_DATA_IMU_OFFSET + 1] << 8))) / 100.0;
        imudata.roll = ((float)(int16_t)(mcuPkt.data[Mcu::REQ_DATA_IMU_OFFSET + 2] +
                           (mcuPkt.data[Mcu::REQ_DATA_IMU_OFFSET + 3] << 8))) / 100.0;
        imudata.yaw = ((float)(int16_t)(mcuPkt.data[Mcu::REQ_DATA_IMU_OFFSET + 4] +
                          (mcuPkt.data[Mcu::REQ_DATA_IMU_OFFSET + 5] << 8))) / 10.0;
        NormalizeIMU(imudata);
        imuPublisher.publish(imudata);
        HeadAngles headA;
        headA.time = get_clock()+30;
        headA.yaw = maxwell->get_joint(6)->current_deg;
        headA.pitch = maxwell->get_joint(5)->current_deg;
        headPublisher.publish(headA);
        
        if (req_cnt % 50 == 0) {
          Mcu::McuPacket pkt;
          pkt.type = Mcu::LED_DATA;
          pkt.len = 2;
          pkt.data[0] = led_status;
          pkt.data[1] = led_status;
          led_status = 1 - led_status;
          mcu->writePacket(pkt);
        }
        if (mcuPkt.data[0] == 1) {
          if (!dxl_connected) {
            imu_init_yaw = imudata.yaw;
            dxl_connected = true;
          }
          SendJointToMCU();
        } else {
          dxl_connected = false;
        }
      }
    }
    ros::spinOnce();
    // loop_rate.sleep();
  }
  mcu->close();
  return 0;
}

void SendJointToMCU() {
  bodyMutex.lock();
  if (!bodyAngles.empty()) {
    maxwell->set_body(bodyAngles.front());
    bodyAngles.pop();
  }
  bodyMutex.unlock();
  headMutex.lock();
  if (!headAngles.empty()) {
    maxwell->set_head(headAngles.front());
    headAngles.pop();
  }
  headMutex.unlock();
  Mcu::McuPacket pkt;
  int i = 0;
  pkt.type = Mcu::MOTOR_DATA;
  pkt.len = maxwell->joint_count() * 5;
  for (auto &jt : maxwell->get_joint_map()) {
    JointPtr &joint = jt.second;
    float deg = (joint->current_deg + joint->offset) * joint->inverse;
    uint32_t gpos = float2dxl(deg);
    pkt.data[i * 5 + 0] = static_cast<uint8_t>(joint->jid);
    pkt.data[i * 5 + 1] = SEU_LOBYTE(SEU_LOWORD(gpos));
    pkt.data[i * 5 + 2] = SEU_HIBYTE(SEU_LOWORD(gpos));
    pkt.data[i * 5 + 3] = SEU_LOBYTE(SEU_HIWORD(gpos));
    pkt.data[i * 5 + 4] = SEU_HIBYTE(SEU_HIWORD(gpos));
    i++;
  }
  mcu->writePacket(pkt);
}

void NormalizeIMU(ImuData &idata) {
  idata.yaw = normalizeDeg<float>(idata.yaw - imu_init_yaw);
  if (idata.pitch < -30.0)
    idata.fall = ImuData::FALL_BACKWARD;
  else if (idata.pitch > 40.0)
    idata.fall = ImuData::FALL_FORWARD;
  else if (idata.roll < -40.0)
    idata.fall = ImuData::FALL_RIGHT;
  else if (idata.roll > 40.0)
    idata.fall = ImuData::FALL_LEFT;
  else
    idata.fall = ImuData::FALL_NONE;
}

bool AddAnglesService(AddAngles::Request &req, AddAngles::Response &res) {
  if (req.part == "body") {
    bodyMutex.lock();
    bodyAngles.push(req.body);
    bodyMutex.unlock();
  } else if (req.part == "head") {
    bodyMutex.lock();
    headAngles.push(req.head);
    bodyMutex.unlock();
  }
}

bool RobotService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  return true;
}

bool GetBoneLengthService(BoneLength::Request &req, BoneLength::Response &res) {
  robot::BonePtr bone = maxwell->get_bone(req.name);
  if (!bone.get()) {
    ROS_ERROR("can not find bone named: %s", req.name.c_str());
    return false;
  }
  res.length = bone->length;
  return true;
}

bool GetQueueSizeService(QueueSize::Request &req, QueueSize::Response &res) {
  if (req.name == "body") {
    bodyMutex.lock();
    res.size = bodyAngles.size();
    bodyMutex.unlock();
  } else if (req.name == "head") {
    headMutex.lock();
    res.size = headAngles.size();
    headMutex.unlock();
  }
  return true;
}

bool InverseKinematicsService(Kinematics::Request &req, Kinematics::Response &res) {
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
  bool ok = false;

  if (req.part == req.LEFT_FOOT) {
    endMat = maxwell->get_foot_mat_from_pose(endPose, true);
    ok = maxwell->leg_inverse_kinematics(bodyMat, endMat, degs, true);
  } else if (req.part == req.RIGHT_FOOT) {
    endMat = maxwell->get_foot_mat_from_pose(endPose, false);
    ok = maxwell->leg_inverse_kinematics(bodyMat, endMat, degs, false);
  } else if (req.part == req.LEFT_HAND || req.part == req.RIGHT_HAND) {
    Vector3d hand;
    hand.x() = endPose.x;
    hand.y() = endPose.y;
    hand.z() = endPose.z;
    ok = maxwell->arm_inverse_kinematics(hand, degs);
  }
  res.success = ok;
  res.degs.resize(degs.size());
  for (int i = 0; i < degs.size(); i++)
    res.degs[i] = degs[i];
  return true;
}

bool GetAnglesService(GetAngles::Request &req, GetAngles::Response &res) {
  if (!dxl_connected) {
    bodyMutex.lock();
    if (!bodyAngles.empty()) {
      maxwell->set_body(bodyAngles.front());
      bodyAngles.pop();
    }
    bodyMutex.unlock();
    headMutex.lock();
    if (!headAngles.empty()) {
      maxwell->set_head(headAngles.front());
      headAngles.pop();
    }
    headMutex.unlock();
  }
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
