#include <QApplication>
#include "action_monitor.hpp"
#include <common/VirtualRobot.h>

using namespace std;
using namespace robot;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_monitor");
    ros::NodeHandle node;
    QApplication app(argc, argv);
    glutInit(&argc, argv);
    ActionMonitor foo(node);
    foo.show();
    return app.exec();
}

ActionMonitor::ActionMonitor(ros::NodeHandle &n): node(n)
{
    setAttribute(Qt::WA_DeleteOnClose);
    std::string robot_file, offset_file;
    try{
        ros::param::get("robot_file", robot_file);
        ros::param::get("offset_file", offset_file);
    }
    catch(ros::InvalidNameException &e){
        ROS_ERROR("%s", e.what());
        exit(0);
    }
    rbt = make_shared<Robot>(robot_file, offset_file);
    rgl = new RobotGL(rbt->get_main_bone(), rbt->get_joint_map());
    btnCtrl = new QPushButton("Start");
    QWidget *mainWidget = new QWidget();
    QVBoxLayout *mainLayout = new QVBoxLayout();
    mainLayout->addWidget(rgl);
    mainLayout->addWidget(btnCtrl);
    mainWidget->setLayout(mainLayout);
    setCentralWidget(mainWidget);

    timer = new QTimer;
    timer->start(20);
    connect(timer, &QTimer::timeout, this, &ActionMonitor::procTimer);
    connect(btnCtrl, &QPushButton::clicked, this, &ActionMonitor::procBtnCtrl);
    start = false;
}

void ActionMonitor::procBtnCtrl()
{
    if(start)
    {
        btnCtrl->setText("Start");
        start = false;
    }
    else
    {
        btnCtrl->setText("Stop");
        start = true;
    }
}

void ActionMonitor::procTimer()
{
    if(start)
    {
        common::VirtualRobot vir;
        ros::service::call("/virtualrobot", vir);
        rbt->set_head(vir.response.head);
        rbt->set_body(vir.response.body);
        std::map<int, float> jds;
        for(auto &j: rbt->get_joint_map())
        {
            jds[j.second->jid] = j.second->current_deg;
        }
        rgl->turn_joint(jds);
    }
    if(!ros::ok()) this->close();
}
