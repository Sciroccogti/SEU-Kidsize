#include <ros/ros.h>
#include <memory>
#include <common/BodyAngles.h>
#include "action/ActionEngine.hpp"
#include "walk/WalkEngine.hpp"
#include <common/BodyTask.h>
#include <common/QueueSize.h>


std::shared_ptr<ActionEngine> actionEng;
std::shared_ptr<WalkEngine> walkEng;

void StartpWalk(bool &isWalking, double &phase, double &time);
void StoppWalk(bool &isWalking, double &phase, double &time);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion");
    ros::NodeHandle node;
    ros::Publisher bodyPublisher = node.advertise<common::BodyAngles>("/bodyangles", 1);
    actionEng = std::make_shared<ActionEngine>(bodyPublisher);
    walkEng = std::make_shared<WalkEngine>(bodyPublisher);

    Eigen::Vector3d step(0.0, 0.0, 0.0);
    double phase = 0.0, time = 0.0; 
    bool isWalking=false;
    common::QueueSize bodyQue;
    bodyQue.request.name = "body";
    while (ros::ok())
    {
        ros::service::call("/bodyangles", bodyQue);
        if(bodyQue.response.size>5) usleep(5000);
        else{
            walkEng->runWalk(step, 2, phase, time);
        }
    }
    return 0;
}

void StartpWalk(bool &isWalking, double &phase, double &time)
{
    isWalking=true;
    Eigen::Vector3d step(0.0, 0.0, 0.0);
    walkEng->runWalk(step, 2, phase, time);
}

void StoppWalk(bool &isWalking, double &phase, double &time)
{
    isWalking=false;
    Eigen::Vector3d step(0.0, 0.0, 0.0);
    walkEng->runWalk(step, 0, phase, time);
}