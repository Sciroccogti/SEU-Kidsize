#include <ros/ros.h>
#include <memory>
#include <mutex>
#include <common/BodyAngles.h>
#include "scan/ScanEngine.hpp"
#include "action/ActionEngine.hpp"
#include "walk/WalkEngine.hpp"
#include <common/BodyTask.h>
#include <common/HeadTask.h>
#include <common/QueueSize.h>

using namespace Eigen;

void StartWalk(bool &isWalking, double &phase, double &time);
void StopWalk(bool &isWalking, double &phase, double &time);

void UpdateBodyTask(const common::BodyTask::ConstPtr &p);
common::BodyTask GetBodyTask();
void UpdateHeadTask(const common::HeadTask::ConstPtr &p);
common::HeadTask GetHeadTask();

std::shared_ptr<ActionEngine> actionEng;
std::shared_ptr<WalkEngine> walkEng;
std::shared_ptr<ScanEngine> scanEng;

common::BodyTask bodyTask;
std::mutex bodyMutex;
common::HeadTask headTask;
std::mutex headMutex;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion");
    ros::NodeHandle node;
    ros::service::waitForService("/maxwell");
    ros::service::waitForService("/addangles");
    actionEng = std::make_shared<ActionEngine>();
    walkEng = std::make_shared<WalkEngine>();
    scanEng = std::make_shared<ScanEngine>();

    ros::Subscriber bodySub = node.subscribe("/bodytask", 1, UpdateBodyTask);
    ros::Subscriber headSub = node.subscribe("/headtask", 1, UpdateHeadTask);

    double phase = 0.0, time = 0.0; 
    bool isWalking=false;
    common::QueueSize bodyQue, headQue;
    bodyQue.request.name = "body";
    headQue.request.name = "head";
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::service::call("/queuesize", headQue);
        if(headQue.response.size<5)
        {
            auto task = GetHeadTask();
            scanEng->runScan(task);
        }
        ros::service::call("/queuesize", bodyQue);
        if(bodyQue.response.size<5)
        {
            auto task = GetBodyTask();
            if(task.type == common::BodyTask::TASK_WALK)
            {
                if(task.count>0)
                {
                    if(!isWalking)
                    {
                        StartWalk(isWalking, phase, time);
                    }
                    else{
                        walkEng->runWalk(Vector3d(task.step, task.lateral, task.turn), task.count, phase, time);
                    }
                }
                else
                {
                    if(isWalking)
                    {
                        StopWalk(isWalking, phase, time);
                    }
                }
            }
            else if(task.type == common::BodyTask::TASK_ACT)
            {
                if(task.count >= 0)
                {
                    if(isWalking)
                        StopWalk(isWalking, phase, time);
                    actionEng->runAction(task.actname);
                }
            }
            else
            {
                StartWalk(isWalking, phase, time);
            }
        }
        loop_rate.sleep();
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
    bodyTask.count = -1;
    auto ret = bodyTask;
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

void StartWalk(bool &isWalking, double &phase, double &time)
{
    isWalking=true;
    Eigen::Vector3d step(0.0, 0.0, 0.0);
    walkEng->runWalk(step, 2, phase, time);
}

void StopWalk(bool &isWalking, double &phase, double &time)
{
    isWalking=false;
    Eigen::Vector3d step(0.0, 0.0, 0.0);
    walkEng->runWalk(step, 0, phase, time);
    phase = 0.0;
    time = 0.0;
}