#ifndef __ACTION_MONITOR_HPP
#define __ACTION_MONITOR_HPP

#include <QtWidgets>
#include "RobotGL.hpp"
#include <robot/robot.hpp>
#include <ros/ros.h>

class ActionMonitor: public QMainWindow
{
    Q_OBJECT
public:
    ActionMonitor(ros::NodeHandle &n);

public slots:
    void procTimer();
    void procBtnCtrl();
    
private:
    ros::NodeHandle &node;
    QPushButton *btnCtrl;
    RobotGL *rgl;
    QTimer *timer;
    std::shared_ptr<robot::Robot> rbt;
    bool start;
};

#endif
