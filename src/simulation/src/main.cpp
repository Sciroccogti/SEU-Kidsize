#include <ros/ros.h>
#include <ros/package.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>  
#include <stdlib.h>  
#include <stdio.h>  
#include <string.h>
#include "unirobot.hpp"

using namespace std;

int main(int argc, char **argv)
{
    setenv("WEBOTS_ROBOT_NAME", "maxwell", 0);
    ros::init(argc, argv, "maxwell");
    ros::NodeHandle node;
    bool ok=false;
    int i=0;
    while(ros::ok() && i++<20)
    {
      FILE   *stream;  
      FILE    *wstream;
      char   buf[1024]; 
      memset( buf, '\0', sizeof(buf) );//初始化buf
      stream = popen( "ls /tmp | grep webots-" , "r" );
      fread( buf, sizeof(char), sizeof(buf),  stream);  //将刚刚FILE* stream的数据流读取到buf中
      pclose( stream ); 
      string sbuf(buf);
      if(sbuf.find("webots") != string::npos)
      {
        string pf = "/tmp/"+sbuf.substr(0, sbuf.find("\n"))+"/WEBOTS_SERVER";
        ROS_WARN("%s", pf.c_str());
        if(access(pf.c_str(), F_OK)!=-1)
        {
          ok = true;
          break;
        }
      }
      ROS_WARN("waiting for webots ......");
      usleep(1000000);
    }
    if(!ok) return 0;
    std::string actpath = ros::package::getPath("simulation")+"/conf/actions";
    shared_ptr<UniRobot> robot = make_shared<UniRobot>(&node, "maxwell", actpath);
    robot->run();
    return 0;
}