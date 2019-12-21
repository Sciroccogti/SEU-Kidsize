#include <common/HeadAngles.h>
#include <sensor_msgs/image_encodings.h>
#include <cstdlib>
#include <fstream>
#include <seumath/math.hpp>
#include "SimRobot.hpp"

using namespace std;
using namespace webots;
using namespace common;

const char *Neck[2] = { "Neck", "Neck2" };
const char *LeftArmJoint[2] = { "LeftShoulder", "LeftElbow" };
const char *RightArmJoint[2] = { "RightShoulder", "RightElbow" };
const char *LeftLegJoint[6] = { "LeftLegX", "LeftLegY", "LeftLegZ", "LeftKnee", "LeftAnkleX", "LeftAnkleY" };
const char *RightLegJoint[6] = { "RightLegX", "RightLegY", "RightLegZ", "RightKnee", "RightAnkleX", "RightAnkleY" };
const double fall_thresh = 1.2;

SimRobot::SimRobot(ros::NodeHandle *node) : Robot()
{
  totalTime = 0;
  mTimeStep = getBasicTimeStep();
  mCamera = getCamera("Camera");
  mCamera->enable(5 * mTimeStep);
  mIMU = getInertialUnit("IMU");
  mIMU->enable(mTimeStep);
  mLEDs.resize(2);
  mLEDs[0] = getLED("Led0");
  mLEDs[1] = getLED("Led1");

  fall_type = ImuData::FALL_NONE;
  mInitYaw = 0.0;
  mYaw = 0.0;

  mHAngles.yaw = 0.0;
  mHAngles.pitch = 0.0;
  mNode = node;
  mImagePublisher = mNode->advertise<sensor_msgs::Image>("/sensor/image", 1);
  mHeadPublisher = mNode->advertise<common::HeadAngles>("/sensor/head", 1);
  mImuPublisher = mNode->advertise<common::ImuData>("/sensor/imu", 1);
}

void SimRobot::PublishImage()
{
  const unsigned char *data = mCamera->getImage();
  int h = mCamera->getHeight(), w = mCamera->getWidth();

  if (data != NULL)
  {
    sensor_msgs::Image image;
    image.header.stamp = ros::Time::now();
    image.header.frame_id = "camera";
    image.width = w;
    image.height = h;
    image.step = 3 * w;
    image.encoding = sensor_msgs::image_encodings::RGB8;
    image.data.resize(3 * w * h);
    for (int i = 0; i < h; i++)
    {
      for (int j = 0; j < w; j++)
      {
        image.data[i * w * 3 + j * 3 + 0] = *(data + i * w * 4 + j * 4 + 2);
        image.data[i * w * 3 + j * 3 + 1] = *(data + i * w * 4 + j * 4 + 1);
        image.data[i * w * 3 + j * 3 + 2] = *(data + i * w * 4 + j * 4 + 0);
      }
    }
    mImagePublisher.publish(image);
  }
}

int SimRobot::myStep()
{
  common::HeadAngles head;
  head.yaw = seumath::rad2deg(mHAngles.yaw);
  head.pitch = seumath::rad2deg(mHAngles.pitch);
  mHeadPublisher.publish(head);
  setPositions();
  checkFall();
  totalTime += mTimeStep;
  if (totalTime % (5 * mTimeStep) == 0)
    PublishImage();
  return step(mTimeStep);
}

void SimRobot::checkFall()
{
  const double *rpy = mIMU->getRollPitchYaw();
  if (rpy[1] > fall_thresh)
    fall_type = ImuData::FALL_BACKWARD;
  else if (rpy[1] < -fall_thresh)
    fall_type = ImuData::FALL_FORWARD;
  else if (rpy[0] < -fall_thresh)
    fall_type = ImuData::FALL_LEFT;
  else if (rpy[0] > fall_thresh)
    fall_type = ImuData::FALL_RIGHT;
  else
    fall_type = ImuData::FALL_NONE;
  mYaw = seumath::normalizeRad<double>(rpy[2] - mInitYaw);
  ImuData imu;
  imu.yaw = seumath::rad2deg(mYaw);
  imu.pitch = rpy[1];
  imu.roll = rpy[0];
  imu.fall = fall_type;
  imu.stamp = ros::Time::now().toNSec();
  mImuPublisher.publish(imu);
  // printf("roll=%f, pitch=%f, yaw=%f\n", rpy[0], rpy[1], rpy[2]);
  // printf("%d\n", fall_type);
}

void SimRobot::wait(int ms)
{
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

void SimRobot::setPositions()
{
  Motor *motor;
  motor = getMotor(LeftLegJoint[0]);
  motor->setPosition(seumath::deg2rad(mAngles.left_hip_roll));
  motor = getMotor(LeftLegJoint[1]);
  motor->setPosition(seumath::deg2rad(mAngles.left_hip_pitch));
  motor = getMotor(LeftLegJoint[2]);
  motor->setPosition(seumath::deg2rad(-mAngles.left_hip_yaw));
  motor = getMotor(LeftLegJoint[3]);
  motor->setPosition(seumath::deg2rad(mAngles.left_knee));
  motor = getMotor(LeftLegJoint[4]);
  motor->setPosition(seumath::deg2rad(mAngles.left_ankle_roll));
  motor = getMotor(LeftLegJoint[5]);
  motor->setPosition(seumath::deg2rad(mAngles.left_ankle_pitch));

  motor = getMotor(RightLegJoint[0]);
  motor->setPosition(seumath::deg2rad(mAngles.right_hip_roll));
  motor = getMotor(RightLegJoint[1]);
  motor->setPosition(seumath::deg2rad(mAngles.right_hip_pitch));
  motor = getMotor(RightLegJoint[2]);
  motor->setPosition(seumath::deg2rad(-mAngles.right_hip_yaw));
  motor = getMotor(RightLegJoint[3]);
  motor->setPosition(seumath::deg2rad(mAngles.right_knee));
  motor = getMotor(RightLegJoint[4]);
  motor->setPosition(seumath::deg2rad(mAngles.right_ankle_roll));
  motor = getMotor(RightLegJoint[5]);
  motor->setPosition(seumath::deg2rad(mAngles.right_ankle_pitch));

  motor = getMotor(LeftArmJoint[0]);
  motor->setPosition(seumath::deg2rad(mAngles.left_shoulder));
  motor = getMotor(LeftArmJoint[1]);
  motor->setPosition(seumath::deg2rad(mAngles.left_elbow));
  motor = getMotor(RightArmJoint[0]);
  motor->setPosition(seumath::deg2rad(-mAngles.right_shoulder));
  motor = getMotor(RightArmJoint[1]);
  motor->setPosition(seumath::deg2rad(-mAngles.right_elbow));

  motor = getMotor(Neck[1]);
  motor->setPosition(seumath::deg2rad(-mHAngles.pitch));
  motor = getMotor(Neck[0]);
  motor->setPosition(seumath::deg2rad(mHAngles.yaw));
}


