#include "unirobot.hpp"
#include <cstdlib>
#include <fstream>
#include <sensor_msgs/image_encodings.h>
#include <seumath/math.hpp>

using namespace std;
using namespace webots;
using namespace common;

const double engineFrequency = 50.0;
const char *Neck[2] = {"Neck", "Neck2"};
const char *LeftArmJoint[2] = {"LeftShoulder", "LeftElbow"};
const char *RightArmJoint[2] = {"RightShoulder", "RightElbow"};
const char *LeftLegJoint[6] = {"LeftLegX", "LeftLegY", "LeftLegZ", "LeftKnee", "LeftAnkleX", "LeftAnkleY"};
const char *RightLegJoint[6] = {"RightLegX", "RightLegY", "RightLegZ", "RightKnee", "RightAnkleX", "RightAnkleY"};
const double fall_thresh = 1.2;
  
UniRobot::UniRobot(ros::NodeHandle *node, string name, std::string actdir): Robot()
{
    mName = name;
    mActdir = actdir+"/";
    totalTime = 0;
    mTimeStep = getBasicTimeStep();
    mCamera = getCamera("Camera");
    mCamera->enable(5 * mTimeStep);
    mIMU= getInertialUnit("IMU");
    mIMU->enable(mTimeStep);
    mLEDs.resize(2);
    mLEDs[0] = getLED("Led0");
    mLEDs[0]->set(0xFF0000);
    mLEDs[0] = getLED("Led1");
    mLEDs[0]->set(0x0000FF);
    fall_type = ImuData::FALL_NONE;
    mInitYaw = 0.0;
    mYaw = 0.0;
    walkInit();
    mHAngles.yaw = 0.0;
    mHAngles.pitch = -M_PI/4;
    mNode = node;
    mImagePublisher = mNode->advertise<sensor_msgs::Image>("/camera/image", 1);
    mImuPublisher = mNode->advertise<sensor_msgs::Image>("/imu", 1);
    mCameraInfoServer = mNode->advertiseService("/camerainfo", &UniRobot::CameraInfoService, this);
}

UniRobot::~UniRobot(){
}

bool UniRobot::CameraInfoService(common::CameraInfo::Request &req, common::CameraInfo::Response &res)
{
    res.camera_type = CameraInfo::Response::CAMERA_RGB;
    res.height = mCamera->getHeight();
    res.width = mCamera->getWidth();
    return true;
}

void UniRobot::PublishImage()
{
    const unsigned char *data = mCamera->getImage();
    int h = mCamera->getHeight(), w = mCamera->getWidth();

    if(data!=NULL)
    {
        sensor_msgs::Image image;
        image.header.stamp = ros::Time::now();
        image.header.frame_id = "camera";
        image.width = w;
        image.height = h;
        image.step = 3*w;
        image.encoding = sensor_msgs::image_encodings::RGB8;
        image.data.resize(3*w*h);
        for(int i=0; i<h; i++)
        {
            for(int j=0; j<w; j++)
            {
                image.data[i*w*3+j*3+0] = *(data+i*w*4+j*4+2);
                image.data[i*w*3+j*3+1] = *(data+i*w*4+j*4+1);
                image.data[i*w*3+j*3+2] = *(data+i*w*4+j*4+0);
            }
        }
        mImagePublisher.publish(image);
    }
}

int UniRobot::myStep() {
    setPositions();
    checkFall();
    ros::spinOnce();
    totalTime += mTimeStep;
    if(totalTime%(5*mTimeStep)==0)
        PublishImage();
    return step(mTimeStep);
}

void UniRobot::checkFall(){
    const double *rpy = mIMU->getRollPitchYaw();
    if(rpy[1]>fall_thresh) fall_type = ImuData::FALL_BACKWARD;
    else if(rpy[1]<-fall_thresh) fall_type = ImuData::FALL_FORWARD;
    else if(rpy[0]<-fall_thresh) fall_type = ImuData::FALL_LEFT;
    else if(rpy[0]>fall_thresh) fall_type = ImuData::FALL_RIGHT;
    else fall_type = ImuData::FALL_NONE;
    mYaw = seumath::normalizeRad<double>(rpy[2]-mInitYaw);
    ImuData imu;
    imu.yaw = mYaw;
    imu.pitch = rpy[1];
    imu.roll = rpy[0];
    imu.fall = fall_type;
    imu.stamp = ros::Time::now().nsec;
    mImagePublisher.publish(imu);
    //printf("roll=%f, pitch=%f, yaw=%f\n", rpy[0], rpy[1], rpy[2]);
    //printf("%d\n", fall_type);
}

void UniRobot::wait(int ms) {
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

void UniRobot::run(){
    double phase = 0.0;
    double time = 0.0;
    while(ros::ok())
    {
        if(fall_type == ImuData::FALL_FORWARD){
            runAct("front_getup");
        }else if(fall_type == ImuData::FALL_BACKWARD){
            runAct("back_getup");
        }
        else if(fall_type == ImuData::FALL_LEFT || fall_type == ImuData::FALL_RIGHT){
            runAct("side_getup");
        }else
        {
            mWalkParams.enabledGain = 1.0;
            mWalkParams.stepGain = 0.03;
            mWalkParams.lateralGain = 0.0;
            mWalkParams.turnGain = 0.0;
            isWalking = true;
            runWalk(mWalkParams, 2.0, phase, time);
        }
    }
}

void UniRobot::runAct(std::string name)
{
    mBodyAngles = loadAct(mActdir+name);
    BodyAngles lastOuts;
    for(auto angles:mBodyAngles)
    {
        lastOuts = mAngles;
        for(int i=0; i<angles.time; i++)
        {
            
            mAngles.left_ankle_pitch = lastOuts.left_ankle_pitch + i*(angles.left_ankle_pitch-lastOuts.left_ankle_pitch)/angles.time;
            mAngles.left_ankle_roll = lastOuts.left_ankle_roll + i*(angles.left_ankle_roll-lastOuts.left_ankle_roll)/angles.time;
            mAngles.left_hip_pitch = lastOuts.left_hip_pitch + i*(angles.left_hip_pitch-lastOuts.left_hip_pitch)/angles.time;
            mAngles.left_hip_roll = lastOuts.left_hip_roll + i*(angles.left_hip_roll-lastOuts.left_hip_roll)/angles.time;
            mAngles.left_hip_yaw = lastOuts.left_hip_yaw + i*(angles.left_hip_yaw-lastOuts.left_hip_yaw)/angles.time;
            mAngles.left_knee = lastOuts.left_knee + i*(angles.left_knee-lastOuts.left_knee)/angles.time;
            mAngles.right_ankle_pitch = lastOuts.right_ankle_pitch + i*(angles.right_ankle_pitch-lastOuts.right_ankle_pitch)/angles.time;
            mAngles.right_ankle_roll = lastOuts.right_ankle_roll + i*(angles.right_ankle_roll-lastOuts.right_ankle_roll)/angles.time;
            mAngles.right_hip_pitch = lastOuts.right_hip_pitch + i*(angles.right_hip_pitch-lastOuts.right_hip_pitch)/angles.time;
            mAngles.right_hip_roll = lastOuts.right_hip_roll + i*(angles.right_hip_roll-lastOuts.right_hip_roll)/angles.time;
            mAngles.right_hip_yaw = lastOuts.right_hip_yaw + i*(angles.right_hip_yaw-lastOuts.right_hip_yaw)/angles.time;
            mAngles.right_knee = lastOuts.right_knee + i*(angles.right_knee-lastOuts.right_knee)/angles.time;

            mAngles.left_shoulder = mAngles.left_shoulder + i*(angles.left_shoulder-mAngles.left_shoulder)/angles.time;
            mAngles.left_elbow = mAngles.left_elbow + i*(angles.left_elbow-mAngles.left_elbow)/angles.time;
            mAngles.right_shoulder = mAngles.right_shoulder + i*(angles.right_shoulder-mAngles.right_shoulder)/angles.time;
            mAngles.right_elbow = mAngles.right_elbow + i*(angles.right_elbow-mAngles.right_elbow)/angles.time;
            myStep();
        }
    }
    isWalking = false;
}

void UniRobot::resetJoints(bool act)
{
    mAngles.left_ankle_pitch = 0;
    mAngles.left_ankle_roll = 0;
    mAngles.left_hip_pitch = 0;
    mAngles.left_hip_roll = 0;
    mAngles.left_hip_yaw = 0;
    mAngles.left_knee = 0;
    mAngles.right_ankle_pitch = 0;
    mAngles.right_ankle_roll = 0;
    mAngles.right_hip_pitch = 0;
    mAngles.right_hip_roll = 0;
    mAngles.right_hip_yaw = 0;
    mAngles.right_knee = 0;

    mAngles.left_shoulder = 0;
    mAngles.left_elbow = 0;
    mAngles.right_shoulder = 0;
    mAngles.right_elbow = 0;
    mHAngles.yaw = 0;
    mHAngles.pitch = 0;
    if(act) myStep();
}

void UniRobot::stopWalk(double &phase, double &time)
{
    mWalkParams.enabledGain = 0.0;
    mWalkParams.stepGain = 0.0;
    mWalkParams.lateralGain = 0.0;
    mWalkParams.turnGain = 0.0;
    runWalk(mWalkParams, 2.0, phase, time);
    isWalking = false;
}

void UniRobot::setPositions()
{
    Motor *motor;
    motor = getMotor(LeftLegJoint[0]);
    motor->setPosition(mAngles.left_hip_roll);
    motor = getMotor(LeftLegJoint[1]);
    motor->setPosition(mAngles.left_hip_pitch);
    motor = getMotor(LeftLegJoint[2]);
    motor->setPosition(-mAngles.left_hip_yaw);
    motor = getMotor(LeftLegJoint[3]);
    motor->setPosition(mAngles.left_knee);
    motor = getMotor(LeftLegJoint[4]);
    motor->setPosition(mAngles.left_ankle_roll);
    motor = getMotor(LeftLegJoint[5]);
    motor->setPosition(mAngles.left_ankle_pitch);

    motor = getMotor(RightLegJoint[0]);
    motor->setPosition(mAngles.right_hip_roll);
    motor = getMotor(RightLegJoint[1]);
    motor->setPosition(mAngles.right_hip_pitch);
    motor = getMotor(RightLegJoint[2]);
    motor->setPosition(-mAngles.right_hip_yaw);
    motor = getMotor(RightLegJoint[3]);
    motor->setPosition(mAngles.right_knee);
    motor = getMotor(RightLegJoint[4]);
    motor->setPosition(mAngles.right_ankle_roll);
    motor = getMotor(RightLegJoint[5]);
    motor->setPosition(mAngles.right_ankle_pitch);

    motor = getMotor(LeftArmJoint[0]);
    motor->setPosition(mAngles.left_shoulder);
    motor = getMotor(LeftArmJoint[1]);
    motor->setPosition(mAngles.left_elbow);
    motor = getMotor(RightArmJoint[0]);
    motor->setPosition(-mAngles.right_shoulder);
    motor = getMotor(RightArmJoint[1]);
    motor->setPosition(mAngles.right_elbow);

    motor = getMotor(Neck[1]);
    motor->setPosition(mHAngles.pitch);
    motor = getMotor(Neck[0]);
    motor->setPosition(mHAngles.yaw);
}


void UniRobot::runWalk(const Rhoban::IKWalkParameters& params, 
    double timeLength, double& phase, double& time)
{
    Rhoban::IKWalkOutputs louts;
    //Walk engine frequency
    for (double t=0.0;t<=timeLength;t+=1.0/engineFrequency) {
        time += 1.0/engineFrequency;
        bool success = Rhoban::IKWalk::walk(
            params, //Walk parameters
            1.0/engineFrequency, //Time step
            phase, //Current walk phase -will be updated)
            louts); //Result target position (updated)
        if (!success) {
            //The requested position for left or right foot is not feasible
            //(phase is not updated)
            std::cout << time << " Inverse Kinematics error. Position not reachable." << std::endl;
        } else {
            mAngles.left_hip_yaw = louts.left_hip_yaw;
            mAngles.left_hip_roll = louts.left_hip_roll;
            mAngles.left_hip_pitch = louts.left_hip_pitch;
            mAngles.left_knee = louts.left_knee;
            mAngles.left_ankle_pitch = louts.left_ankle_pitch;
            mAngles.left_ankle_roll = louts.left_ankle_roll;
            mAngles.right_hip_yaw = louts.right_hip_yaw;
            mAngles.right_hip_roll = louts.right_hip_roll;
            mAngles.right_hip_pitch = louts.right_hip_pitch;
            mAngles.right_knee = louts.right_knee;
            mAngles.right_ankle_pitch = louts.right_ankle_pitch;
            mAngles.right_ankle_roll = louts.right_ankle_roll;
            mAngles.left_shoulder = 0.06352998;
            mAngles.left_elbow = -2.93843628;
            mAngles.right_shoulder = 0.06352998;
            mAngles.right_elbow = -2.93843628;
            myStep();
        }
    }
}

std::vector<BodyAngles> UniRobot::loadAct(std::string name)
{
    ifstream ifs(name);
    if(!ifs)
    {
        printf("%s: can't find act [%s]\n", mName.c_str(), name.c_str());
        return {};
    }
    std::vector<BodyAngles> ret;
    BodyAngles angles;
    while(!ifs.eof()){
        ifs >> angles.time;
        ifs >> angles.right_hip_yaw >> angles.right_hip_roll >> angles.right_hip_pitch 
            >> angles.right_knee >> angles.right_ankle_pitch >> angles.right_ankle_roll;
        ifs >> angles.left_hip_yaw >> angles.left_hip_roll >> angles.left_hip_pitch 
            >> angles.left_knee >> angles.left_ankle_pitch >> angles.left_ankle_roll;
        ifs >> angles.right_shoulder >> angles.right_elbow
            >> angles.left_shoulder >> angles.left_elbow;
        ret.push_back(angles);
    }
    return ret;
}

void UniRobot::walkInit()
{
  mWalkParams.distHipToKnee = 0.15;
  mWalkParams.distKneeToAnkle = 0.15;
  mWalkParams.distAnkleToGround = 0.03;
  mWalkParams.distFeetLateral = 0.1098;
  mWalkParams.freq = 2.0;
  mWalkParams.enabledGain = 1.0;
  mWalkParams.supportPhaseRatio = 0.0;
  mWalkParams.footYOffset = 0.02;
  mWalkParams.stepGain = 0.0;
  mWalkParams.riseGain = 0.05;
  mWalkParams.turnGain = 0.0;
  mWalkParams.lateralGain = 0.0;
  mWalkParams.trunkZOffset = 0.02;
  mWalkParams.swingGain = 0.02;
  mWalkParams.swingRollGain = 0.0;
  mWalkParams.swingPhase = 0.25;
  mWalkParams.stepUpVel = 3.0;
  mWalkParams.stepDownVel = 3.0;
  mWalkParams.riseUpVel = 3.0;
  mWalkParams.riseDownVel = 3.0;
  mWalkParams.swingPause = 0.0;
  mWalkParams.swingVel = 4.0;
  mWalkParams.trunkXOffset = 0.02;
  mWalkParams.trunkYOffset = 0.0;
  mWalkParams.trunkPitch = 0.15;
  mWalkParams.trunkRoll = 0.0;
  mWalkParams.extraLeftX = 0.0;
  mWalkParams.extraLeftY = 0.0;
  mWalkParams.extraLeftZ = 0.0;
  mWalkParams.extraRightX = 0.0;
  mWalkParams.extraRightY = 0.0;
  mWalkParams.extraRightZ = 0.0;
  mWalkParams.extraLeftYaw = 0.0;
  mWalkParams.extraLeftPitch = 0.0;
  mWalkParams.extraLeftRoll = 0.0;
  mWalkParams.extraRightYaw = 0.0;
  mWalkParams.extraRightPitch = 0.0;
  mWalkParams.extraRightRoll = 0.0;
}