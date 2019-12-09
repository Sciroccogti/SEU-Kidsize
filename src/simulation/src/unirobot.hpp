#ifndef __UNIROBOT_HPP
#define __UNIROBOT_HPP

#include <ros/ros.h>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/LED.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/PositionSensor.hpp>
#include <sensor_msgs/Image.h>
#include <common/BodyAngles.h>
#include <common/HeadAngles.h>
#include <common/CameraInfo.h>
#include <common/ImuData.h>
#include "walk/IKWalk.hpp"

class UniRobot: public webots::Robot
{
public:
    UniRobot(ros::NodeHandle *node, std::string name, std::string actdir);
    ~UniRobot();
    void run();

private:
    int mTimeStep;
    int totalTime;
    std::string mName, mActdir;

    ros::NodeHandle *mNode;
    ros::ServiceServer mCameraInfoServer;
    ros::Publisher mImagePublisher;
    ros::Publisher mImuPublisher;

    bool CameraInfoService(common::CameraInfo::Request &req, common::CameraInfo::Response &res);
    void PublishImage();

    std::vector<common::BodyAngles> mBodyAngles;
    Rhoban::IKWalkParameters mWalkParams;
    bool isWalking;
    void walkInit();
    void runWalk(const Rhoban::IKWalkParameters& params, double timeLength, double& phase, double& time);
    void stopWalk(double &phase, double &time);
    void runAct(std::string name);
    void resetJoints(bool act=true);
    void setPositions();


    void checkFall();
    std::vector<common::BodyAngles> loadAct(std::string name);
    
    int myStep();
    void wait(int ms);

    webots::Camera *mCamera;
    webots::InertialUnit *mIMU;
    std::vector<webots::LED*> mLEDs;

    int fall_type;
    double mYaw, mInitYaw;

    common::BodyAngles mAngles;
    common::HeadAngles mHAngles;
};


#endif
