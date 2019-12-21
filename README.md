# 基于ROS的机器人控制系统
适用于SEU-UniRobot的机器人控制系统，包含机器人运动控制，机器人图像处理，机器人决策，机器人调试工具，机器人仿真系统等；主要开发语言为C/C++，脚本语言为Python。
## 依赖项  
- 编译环境  
    + CMake >= 3.14  
    + gcc/g++  
    + nvcc

- C++ 
    + ROS  
    + CUDA >= 9.0  
    + cuDNN >= 7.0   
    + libeigen3-dev
    + MVSDK (摄像头SDK)  

- Python
    + paramiko  

- 仿真环境  
    + Webots(安装位置必须是/usr/local/webots)  
    + 需要在～/.bashrc中添加： export WEBOTS_HOME=/usr/local/webots  


## 开发工具   
+ VS Code (插件: C/C++, Python, ROS, CMake)  
+ astyle (代码格式化工具)  

## 目录说明  
+ common: 数据结构定义  
+ communication: 比赛控制器通信  
+ config: 配置文件存放及解析  
+ libraries: 自己实现的库  
    - seuimage: 图像处理相关  
    - seumath: 基于Eigen的数学库  
    - robot: 机器人相关功能  
    - darknet: 神经网络框架  
+ motion: 机器人运动控制  
+ player: 机器人控制器  
+ vision: 机器人视觉  
+ control: 机器人决策控制  
+ simulation: 基于Webots的仿真系统  
+ tools: 机器人调试工具  
+ [start](src/start/start.md): 启动文件 
