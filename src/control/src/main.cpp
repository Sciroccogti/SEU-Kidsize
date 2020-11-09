#include <common/BodyTask.h>
#include <common/GcInfo.h>
#include <common/HeadAngles.h>
#include <common/HeadTask.h>
#include <common/ImageResult.h>
#include <common/ImuData.h>
#include <common/LedTask.h>
#include <common/PlayerInfo.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <common/datadef.hpp>
#include <config/basic_parser.hpp>
#include <fstream>
#include <mutex>

using namespace common;

const char *GameState[]{"Initial", "Ready", "Set", "Playing", "Finish"};

// searchBallTable = [(-90, 0), (-45, 0), (0, 0), (45, 0), (90, 0),
//     (90, 30), (45, 30), (0, 30), (-45, 30), (-90, 30),
//     (-70, 55), (-35, 55), (0, 55), (35, 55), (70, 55)]
// searchPostTable = [(-90, 0), (-45, 0), (0, 0), (45, 0), (90, 0),
//     (90, 30), (45, 30), (0, 30), (-45, 30), (-90, 30)]

ImageResult imgResult;
HeadAngles headAngles;
ImuData imuData;
GcInfo gcData;

void imgResUpdate(const ImageResult::ConstPtr &p) { imgResult = *p; }

void headUpdate(const HeadAngles::ConstPtr &p) { headAngles = *p; }

void imuUpdate(const ImuData::ConstPtr &p) { imuData = *p; }

void gcUpdate(const GcInfo::ConstPtr &p) { gcData = *p; }

int main(int argc, char **argv) {
    uint8_t id = 0;  // TODO: read from conf
    int gcstattmp = -1;
    ros::init(argc, argv, "strategy");
    ros::NodeHandle node;
    ros::Publisher bodyTaskPublisher =
        node.advertise<BodyTask>("/task/body", 1);
    ros::Publisher headTaskPublisher =
        node.advertise<HeadTask>("/task/head", 1);
    ros::Publisher ledTaskPublisher = node.advertise<LedTask>("/task/led", 1);
    ros::Publisher playerInfoPublisher =
        node.advertise<PlayerInfo>("/sensor/1", 1);
    ros::Subscriber imgResSubscriber =
        node.subscribe("/result/vision/imgproc", 1, imgResUpdate);
    ros::Subscriber headSubcriber =
        node.subscribe("/sensor/head", 1, headUpdate);
    ros::Subscriber imuSubcriber = node.subscribe("/sensor/imu", 1, imuUpdate);
    ros::Subscriber gcSubscriber = node.subscribe("/sensor/gctrl", 1, gcUpdate);

    // std::string cfgpath = ros::package::getPath("config") + "/conf/";
    // std::string config_str = cfgpath + "config.conf";
    // bpt::ptree pt;
    // get_tree_from_file(config_str, pt);
    // bpt::ptree ptstrategy = pt.get_child("strategy");
    // bpt::ptree ptguard = ptstrategy.get_child("guard");
    const float init_pos[2] = {-3.0, 0.0};
    const float start_pos[2] = {-3.0, 3.0};
    const float attack_range[2] = {-4.5, 0.0};

    ros::Rate rate(10);

    while (ros::ok()) {
        HeadTask htask;
        BodyTask btask;
        PlayerInfo pinfo;
        btask.type = BodyTask::TASK_ACT;
        // btask.count = 2;
        // btask.step = 0.0;
        btask.actname = "ready";
        pinfo.id = id;

        if (gcstattmp != gcData.state) {
            gcstattmp = gcData.state;
            ROS_INFO("Entering %s", GameState[gcstattmp]);
        }

        switch (gcstattmp) {
            case GC_INITIAL:
                btask.type = BodyTask::TASK_ACT;
                btask.actname = "ready";
                break;

            default:
                btask.type = BodyTask::TASK_ACT;
                btask.actname = "reset";
                htask.mode = htask.ModeScanBall;
                break;
        }
        headTaskPublisher.publish(htask);
        bodyTaskPublisher.publish(btask);
        playerInfoPublisher.publish(pinfo);
        ros::spinOnce();
        rate.sleep();
    }

    // rate = rospy.Rate(20)
    // lstatus = True
    // current_state = -1
    // i = 0
    // j = 0
    // while not rospy.is_shutdown():
    //     htask = HeadTask()
    //     btask = BodyTask()
    //     pinfo = PlayerInfo()
    //     GcUpdate()
    //     btask.type = BodyTask.TASK_WALK
    //     btask.count = 2
    //     btask.step = 0.0
    //     pinfo.id = id

    //     if gcData.state != current_state:
    //         current_state = gcData.state
    //         statestr = stateMap[current_state]
    //         rospy.loginfo("Enter %s" % statestr)

    //         if statestr == "Initial":
    //             btask.type = BodyTask.TASK_ACT
    //             btask.actname = "reset"
    //             # TODO: read from guard conf
    //             pinfo.self_x = -3.0
    //             pinfo.self_y = 0.0

    //         if statestr == "Ready":
    //             pass

    //     if imgResult.has_ball:
    //         x = imgResult.ball.x
    //         y = imgResult.ball.y
    //         htask.yaw = searchBallTable[i%len(searchBallTable)][0]
    //         htask.pitch = searchBallTable[i%len(searchBallTable)][1]
    //     else:
    //         if j%15 == 0:
    //             i = i+1
    //         htask.yaw = searchBallTable[i%len(searchBallTable)][0]
    //         htask.pitch = searchBallTable[i%len(searchBallTable)][1]
    //         j = j+1

    //     bodyTaskPublisher.publish(btask)
    //     headTaskPublisher.publish(htask)
    //     playerInfoPublisher.publish(pinfo)
    //     rate.sleep()
}