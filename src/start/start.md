# start 包说明  
该包定义了各种launch文件和参数更新节点  

## 文件说名：
+ src  
    - [params_update.cpp](src/params_update.cpp): 参数更新节点  
+ launch  
    - [update_params.launch](launch/update_params.launch): 参数更新节点的launch文件  
    - [start_camera.launch](launch/start_camera.launch): 摄像头节点的launch文件  
    - [start_imgproc.launch](launch/start_imgproc.launch): 图像处理节点的launch文件  
    - [start_player.launch](launch/start_player.launch): player节点的launch文件  
    - [start_robot.launch](launch/start_robot.launch): 机器人的launch文件，启动所有相关节点     
    - [start_vision.launch](launch/start_vision.launch): 视觉的launch文件，启动视觉相关节点    
    - [start_without_vision.launch](launch/start_without_vision.launch): 机器人的launch文件，但是不启动视觉相关节点    

