#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <common/CameraProperty.h>
#include <common/CameraInfo.h>
#include <common/ImageResult.h>
#include <common/ImuData.h>
#include <common/HeadAngles.h>
#include <common/datadef.hpp>
#include <darknet/network.h>
#include <darknet/parser.h>
#include <opencv2/opencv.hpp>
#include <std_srvs/SetBool.h>
#include <mutex>
#include <atomic>
#include <queue>
#include "vision_parser.hpp"
#include <seuimage/seuimage.hpp>

using namespace seuimage;

bool MallocMemory();
void NetworkInit(std::string cfg, std::string wts);
void GetCameraInfo(ros::NodeHandle &node);
void ImageUpdate(const sensor_msgs::Image::ConstPtr &p);
void ImuUpdate(const common::ImuData::ConstPtr &p);
void HeadUpdate(const common::HeadAngles::ConstPtr &p);
void Run(const ros::TimerEvent& event);

void ImagePublish(const cv::Mat &bgr);
bool SendSrcService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

int camera_width;
int camera_height;
int camera_type;
common::CameraProperty camera_property;
const int width=640;
const int height=480;
std::mutex image_mutex;
std::atomic_bool send_src(false);
cv::Mat cmrMat;
CudaMatC srcMat;
CudaMatC rgbMat;
CudaMatC dstMat;
CudaMatC hsvMat;
CudaMatC relMat;
CudaMatC netMat;
CudaMatF netfMat;
CudaMatF yoloMat;
network yolo;

std::mutex imu_mutex;
common::ImuData imuData;
std::mutex head_mutex;
std::queue<common::HeadAngles> headDatas;
const int headQueueSize = 4;

ros::Publisher imagePublisher;
ros::Publisher resultPublisher;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imgproc");
    ros::NodeHandle node;
    bool params_seted = false;
    while(!params_seted && ros::ok()){
        try{
            ros::param::get("params", params_seted);
        }catch(const std::exception& e){
            ROS_WARN("%s", e.what());
        }
        ROS_INFO("waiting for params set......");
        usleep(1000000);
    }
    GetCameraInfo(node);
    if(camera_type == common::CameraInfo::Response::CAMERA_NONE)
        return 0;
    ROS_INFO("w=%d, h=%d, type=%d", camera_width, camera_height, camera_type);
    std::string cfgfile;
    std::string visionfile;

    try{
        ros::param::get("camera_property_file", cfgfile);
        ros::param::get("vision_file", visionfile);
    }
    catch(ros::InvalidNameException &e){
        ROS_ERROR("%s", e.what());
        return 0;
    }

    parse(cfgfile, camera_property);
    
    std::string vision_root = visionfile.substr(0, visionfile.find_last_of("/")+1);
    std::string yolo_cfg, weights;
    if(!visionfile.empty())
    {
        common::bpt::ptree pt;
        if(!common::get_tree_from_file(visionfile, pt))
        {
            ROS_ERROR("parse file: [%s] failed", visionfile.c_str());
            return 0;
        }
        yolo_cfg = vision_root+pt.get<std::string>("cfg_file");
        weights = vision_root+pt.get<std::string>("weights_file");
    }
    
    NetworkInit(yolo_cfg, weights);

    if(!MallocMemory()){
        ROS_ERROR("MallocMemory failed");
        return 0;
    }
    ros::Subscriber imageSubScriber = node.subscribe("/sensor/image", 1, ImageUpdate);
    ros::Subscriber imuSubScriber = node.subscribe("/sensor/imu", 1, ImuUpdate);
    ros::Subscriber headSubScriber = node.subscribe("/sensor/head", 1, HeadUpdate);
    imagePublisher = node.advertise<sensor_msgs::CompressedImage>("/result/vision/compressed", 1);
    resultPublisher = node.advertise<common::ImageResult>("/result/vision/imgproc", 1);
    ros::ServiceServer typeServer = node.advertiseService("/sendsrc", SendSrcService);
    ros::Timer timer = node.createTimer(ros::Duration(0.1), Run);
    ros::spin();

    return 0;
}

void Run(const ros::TimerEvent& event)
{
    if(!ros::ok()) return;
    bool ret;
    image_mutex.lock();
    ret = srcMat.upload(cmrMat);
    image_mutex.unlock();
    
    if(!ret)
    {
        ROS_ERROR("upload error");
        return;
    }
    if (camera_type == common::CameraInfo::Response::CAMERA_Bayer)
    {
        ret = BayerToRGB(srcMat, rgbMat);
        ret = Resize(rgbMat, dstMat);
        ret = WhiteBalance(dstMat, 1.0, 1.0, 1.0);

        //err = cudaUndistored(dev_target_, dev_undis_, pCamKData, pDistortData, pInvNewCamKData, pMapxData, pMapyData, w_, h_, 3);
        ret = dstMat.copyTo(relMat);
    }
    else if(camera_type == common::CameraInfo::Response::CAMERA_YUYV)
    {
        ret = YUV422ToRGB(srcMat, rgbMat);
        ret = Resize(rgbMat, dstMat);
        ret = dstMat.copyTo(relMat);
    }
    else if(camera_type == common::CameraInfo::Response::CAMERA_RGB)
    {
        ret = Resize(srcMat, dstMat);
        ret = dstMat.copyTo(relMat);
    }
    else return;
    ret = RGBToHSV(relMat, hsvMat);
    
    ret = Resize(relMat, netMat);
    if(!ret)
    {
        ROS_ERROR("Resize to net error");
        return;
    }
    ret = RGB8uTo32fNorm(netMat, netfMat);
    if(!ret)
    {
        ROS_ERROR("RGB8uTo32fNorm error");
        return;
    }
    ret = PackedToPlanar(netfMat, yoloMat);
    if(!ret)
    {
        ROS_ERROR("PackedToPlanar error");
        return;
    }

    layer l = yolo.layers[yolo.n - 1];
    network_predict1(yolo, yoloMat.data());
    int nboxes = 0;
    float nms = 0.45;
    detection *dets = get_network_boxes(&yolo, width, height, 
                    0.5, 0.5, 0, 1, &nboxes, 0);

    if (nms)
    {
        do_nms_sort(dets, nboxes, l.classes, nms);
    }

    std::vector<DetObject> ball_dets, post_dets;
    ball_dets.clear();
    post_dets.clear();

    for (int i = 0; i < nboxes; i++)
    {
        if (dets[i].prob[0] > dets[i].prob[1])
        {
            if (dets[i].prob[0] >= 0.6)
            {
                int bx = (dets[i].bbox.x - dets[i].bbox.w / 2.0) * width;
                int by = (dets[i].bbox.y - dets[i].bbox.h / 2.0) * height;
                int bw = dets[i].bbox.w * width, bh = dets[i].bbox.h * height + 1;
                ball_dets.push_back(DetObject(0, dets[i].prob[0], bx, by, bw, bh));
            }
        }
        else
        {
            if (dets[i].prob[1] >= 0.5)
            {
                int px = (dets[i].bbox.x - dets[i].bbox.w / 2.0) * width;
                int py = (dets[i].bbox.y - dets[i].bbox.h / 2.0) * height;
                int pw = dets[i].bbox.w * width, ph = dets[i].bbox.h * height;
                post_dets.push_back(DetObject(1, dets[i].prob[1], px, py, pw, ph));
            }
        }
    }
    std::sort(ball_dets.rbegin(), ball_dets.rend());
    std::sort(post_dets.rbegin(), post_dets.rend());
    free_detections(dets, nboxes);

    common::ImageResult imgResult;
    if (!ball_dets.empty())
    {
        imgResult.has_ball = true;
        imgResult.ball.x = ball_dets[0].x+ball_dets[0].w/2;
        imgResult.ball.y = ball_dets[0].y+ball_dets[0].h;
    }
    uint32_t posts_cnt = post_dets.size();
    imgResult.posts.resize(posts_cnt>2?2:posts_cnt);
    for (uint32_t i = 0; i < imgResult.posts.size(); i++)
    {
        imgResult.has_post = true;
        imgResult.posts[i].x = ball_dets[i].x+ball_dets[i].w/2;
        imgResult.posts[i].y = ball_dets[i].y+ball_dets[i].h/3*2;
    }
    imu_mutex.lock();
    imgResult.body_yaw = imuData.yaw;
    imu_mutex.unlock();
    head_mutex.lock();
    if(headDatas.size()>0)
    {
        imgResult.head_pitch = headDatas.front().pitch;
        imgResult.head_yaw = headDatas.front().yaw;
        headDatas.pop();
    }
    head_mutex.unlock();
    resultPublisher.publish(imgResult);

    bool dbg;
    ros::param::get("debug", dbg);
    if(dbg)
    {   
        cv::Mat rgb(height, width, CV_8UC3);
        if(!dstMat.download(rgb))
        {
            ROS_ERROR("download error");
            return;
        }
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, CV_RGB2BGR);
        
        if(!send_src)
        {
            if (!ball_dets.empty())
            {
                cv::rectangle(bgr, cv::Point(ball_dets[0].x, ball_dets[0].y), cv::Point(ball_dets[0].x + ball_dets[0].w,
                            ball_dets[0].y + ball_dets[0].h), cv::Scalar(255, 0, 0), 2);
            }

            for (uint32_t i = 0; i < post_dets.size(); i++)
            {
                if (i >= 2)
                {
                    break;
                }

                cv::rectangle(bgr, cv::Point(post_dets[i].x, post_dets[i].y), cv::Point(post_dets[i].x + post_dets[i].w,
                            post_dets[i].y + post_dets[i].h), cv::Scalar(0, 0, 255), 2);
            }
        }
        ImagePublish(bgr);
    }
}

void NetworkInit(std::string cfg, std::string wts)
{
    yolo.gpu_index = 0;
    yolo = parse_network_cfg_custom((char*)(cfg.c_str()), 1, 0);
    load_weights(&yolo, const_cast<char *>((char*)(wts.c_str())));
    set_batch_network(&yolo, 1);
    fuse_conv_batchnorm(yolo);
    calculate_binary_weights(yolo);
    srand(2222222);
}

bool MallocMemory()
{
    bool ret;
    if(camera_type == common::CameraInfo::Response::CAMERA_Bayer)
    {
        cmrMat.create(camera_height, camera_width, CV_8UC1);
        ret = srcMat.create(camera_height, camera_width, 1);
        if(!ret) return false;
    }
    else if(camera_type == common::CameraInfo::Response::CAMERA_YUYV)
    {
        cmrMat.create(camera_height, camera_width, CV_8UC2);
        ret = srcMat.create(camera_height, camera_width, 2);
        if(!ret) return false;
    }
    else if(camera_type == common::CameraInfo::Response::CAMERA_RGB)
    {
        cmrMat.create(camera_height, camera_width, CV_8UC3);
        ret = srcMat.create(camera_height, camera_width, 3);
        if(!ret) return false;
    }
        
    ret = rgbMat.create(camera_height, camera_width, 3);
    if(!ret) return false;

    ret = dstMat.create(height, width, 3);
    if(!ret) return false;

    ret = relMat.create(height, width, 3);
    if(!ret) return false;

    ret = hsvMat.create(height, width, 3);
    if(!ret) return false;

    ret = netMat.create(yolo.h, yolo.w, 3);
    if(!ret) return false;

    ret = netfMat.create(yolo.h, yolo.w, 3);
    if(!ret) return false;

    ret = yoloMat.create(yolo.h, yolo.w, 3);
    if(!ret) return false;
    
    cudaSetDevice(0);
    return true;
}

void GetCameraInfo(ros::NodeHandle &node)
{
    ros::service::waitForService("/camerainfo");
    ros::ServiceClient infoClient = node.serviceClient<common::CameraInfo>("/camerainfo");
    common::CameraInfo info;
    infoClient.call(info);
    camera_type = info.response.camera_type;
    camera_width = info.response.width;
    camera_height = info.response.height;
}

void ImageUpdate(const sensor_msgs::Image::ConstPtr &p)
{
    image_mutex.lock();
    memcpy(cmrMat.data, &(p->data[0]), cmrMat.rows*cmrMat.cols*cmrMat.channels());
    image_mutex.unlock();
}

void ImuUpdate(const common::ImuData::ConstPtr &p)
{
    imu_mutex.lock();
    imuData = *p;
    imu_mutex.unlock();
}

void HeadUpdate(const common::HeadAngles::ConstPtr &p)
{
    head_mutex.lock();
    if(headDatas.size()>= headQueueSize) headDatas.pop();
    headDatas.push(*p);
    head_mutex.unlock();
}

void ImagePublish(const cv::Mat &bgr)
{
    sensor_msgs::CompressedImage image;
    std::vector<uint8_t> buf;
    cv::imencode(".jpg", bgr, buf);
    image.header.stamp = ros::Time::now();
    image.header.frame_id = "vision";
    image.format = "jpeg";
    image.data.resize(buf.size());
    memcpy(&image.data[0], &(buf[0]), buf.size());
    imagePublisher.publish(image);
}

bool SendSrcService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    send_src = req.data;
    return true;
}