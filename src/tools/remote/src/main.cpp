#include <QApplication>
#include "RemoteControl.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "remote_tool");
    ros::NodeHandle node;
    QApplication app(argc, argv);
    RemoteControl foo(node);
    foo.show();
    return app.exec();
}
