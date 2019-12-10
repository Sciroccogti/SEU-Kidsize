#include <string>
#include <sys/mman.h>
#include <errno.h>
#include <functional>
#include <sstream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <CameraApi.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <common/CameraProperty.h>
#include <common/CameraInfo.h>
#include <common/datadef.hpp>
#include "vision_parser.hpp"

struct VideoBuffer
{
    uint8_t *start;
    size_t offset;
    size_t length;
    size_t bytesused;
    int lagtimems;
};

extern bool open_camera(std::string cfile);
extern void close_mv_camera();
extern void close_uvc_camera();
extern void update_camera(const common::CameraProperty::ConstPtr &p);
extern void publish_image(const uint8_t *data);
extern bool info_service(common::CameraInfo::Request &req, common::CameraInfo::Response &res);
extern void mv_run();
extern void uvc_run();

int camera_fd = -1;
int width;
int height;
int camera_type;
common::CameraProperty camera_property;

// for USB Camera
uint8_t *buffers;
uint32_t buf_idx;
v4l2_buffer v4l_buf;

bool is_alive = false;
ros::Publisher imagePublisher;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera");
    ros::NodeHandle node;
    bool params_seted = false;
    while(!params_seted && ros::ok()){
        try{
            ros::param::get("params", params_seted);
        }catch(const std::exception& e){
            ROS_WARN("%s", e.what());
        }
        usleep(100000);
    }
    std::string cfgfile;
    try{
        ros::param::get("camera_property_file", cfgfile);
    }
    catch(ros::InvalidNameException &e){
        ROS_ERROR("%s", e.what());
        return 0;
    }
    if(!open_camera(cfgfile))
    {
        return 0;
    }
    imagePublisher = node.advertise<sensor_msgs::Image>("/sensor/image", 1);
    ros::Subscriber proSubscriber = node.subscribe("/cameraproperty", 1, update_camera);
    ros::ServiceServer infoServer = node.advertiseService("/camerainfo", info_service);
    if (camera_type == common::CameraInfo::Response::CAMERA_Bayer)
    {
        mv_run();
        close_mv_camera();
    }
    else if (camera_type == common::CameraInfo::Response::CAMERA_YUYV)
    {
        uvc_run();
        close_uvc_camera();
    }
    return 0;
}

bool info_service(common::CameraInfo::Request &req, common::CameraInfo::Response &res)
{
    res.camera_type = camera_type;
    res.height = height;
    res.width = width;
    return true;
}

void publish_image(uint8_t *data)
{
    if(camera_type == common::CameraInfo::Response::CAMERA_NONE) return;
    sensor_msgs::Image image;
    image.header.frame_id = "camera";
    image.header.stamp = ros::Time::now();
    image.height = height;
    image.width = width;
    
    if(camera_type == common::CameraInfo::Response::CAMERA_Bayer)
    {
        image.step = width;
        image.encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
        image.data.resize(width*height);
        memcpy(&(image.data[0]), data, width*height);
    }
    else if(camera_type == common::CameraInfo::Response::CAMERA_YUYV)
    {
        image.step = 2*width;
        image.encoding = sensor_msgs::image_encodings::YUV422;
        image.data.resize(2*width*height);
        memcpy(&(image.data[0]), data, 2*width*height);
    }
    imagePublisher.publish(image);
}

void mv_run()
{
    tSdkFrameHead sFrameInfo;
    uint8_t *buffer;

    while (ros::ok())
    {
        CameraSdkStatus status = CameraGetImageBuffer(camera_fd, &sFrameInfo, &buffer, 1000);

        if (status == CAMERA_STATUS_SUCCESS)
        {
            publish_image(buffer);
            CameraReleaseImageBuffer(camera_fd, buffer);
        }
        ros::spinOnce();
        usleep(10000);
    }
}

void uvc_run()
{
    v4l_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l_buf.memory = V4L2_MEMORY_MMAP;

    while (ros::ok())
    {
        if (ioctl(camera_fd, VIDIOC_DQBUF, &v4l_buf) == -1)
        {
            ROS_ERROR("VIDIOC_DQBUF failed.");
            break;
        }

        __u32 idx = v4l_buf.index;
        VideoBuffer *vbuffers = (VideoBuffer *)buffers;
        vbuffers[idx].bytesused = v4l_buf.bytesused;
        vbuffers[idx].length = v4l_buf.length;
        vbuffers[idx].offset = v4l_buf.m.offset;

        if (ioctl(camera_fd, VIDIOC_QBUF, &v4l_buf) == -1)
        {
            ROS_ERROR("VIDIOC_QBUF error");
            is_alive = false;
            break;
        }

        buf_idx = v4l_buf.index;
        publish_image(((VideoBuffer *)buffers)[buf_idx].start);
        ros::spinOnce();
        usleep(10000);
    }
}

void set_camera()
{
    CameraSetAnalogGain(camera_fd, camera_property.exposure_gain);
    CameraSetAnalogGain(camera_fd, camera_property.exposure_time * 1000);
}

void update_camera(const common::CameraProperty::ConstPtr &p)
{
    if(camera_type != common::CameraInfo::Response::CAMERA_Bayer) return;
    camera_property.exposure_gain = p->exposure_gain;
    camera_property.exposure_time = p->exposure_time;
    set_camera();
}

bool open_camera(std::string cfile)
{
    int iCameraCounts = 1;
    int iStatus = -1;
    tSdkCameraDevInfo tCameraEnumList;
    CameraSdkInit(1);
    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);

    if (iCameraCounts == 0)
    {
        ROS_WARN("open MV camera failed!");
    }
    else
    {
        camera_type = common::CameraInfo::Response::CAMERA_Bayer;
    }

    if (camera_type == common::CameraInfo::Response::CAMERA_Bayer)
    {
        bool ret = parse(cfile, camera_property);
        iStatus = CameraInit(&tCameraEnumList, -1, PARAMETER_TEAM_DEFAULT, &camera_fd);

        if (iStatus != CAMERA_STATUS_SUCCESS)
        {
            return false;
        }

        tSdkCameraCapbility     tCapability;
        CameraGetCapability(camera_fd, &tCapability);
        CameraSetAeState(camera_fd, false);
        if(ret)
        {
            set_camera();
        }
        CameraSetImageResolution(camera_fd, &(tCapability.pImageSizeDesc[0]));
        width = tCapability.pImageSizeDesc[0].iWidth;
        height = tCapability.pImageSizeDesc[0].iHeight;
        CameraPlay(camera_fd);
    }
    else
    {
        camera_fd = ::open("/dev/video0", O_RDWR, 0);

        if (camera_fd < 0)
        {
            ROS_ERROR("open camera failed");
            return false;
        }

        width = 640;
        height = 480;
        v4l2_format fmt;
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

        if (ioctl(camera_fd, VIDIOC_S_FMT, &fmt) < 0)
        {
            ROS_ERROR("set format failed");
            return false;
        }

        if (ioctl(camera_fd, VIDIOC_G_FMT, &fmt) < 0)
        {
            ROS_ERROR("get format failed");
            return false;
        }

        v4l2_requestbuffers req;
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (ioctl(camera_fd, VIDIOC_REQBUFS, &req) == -1)
        {
            ROS_ERROR("request buffer error");
            return false;
        }

        buffers = (uint8_t *)calloc(req.count, sizeof(VideoBuffer));

        for (buf_idx = 0; buf_idx < req.count; buf_idx++)
        {
            v4l_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            v4l_buf.memory = V4L2_MEMORY_MMAP;
            v4l_buf.index = buf_idx;

            if (ioctl(camera_fd, VIDIOC_QUERYBUF, &v4l_buf) == -1)
            {
                ROS_ERROR("query buffer error");
                return false;
            }

            VideoBuffer *vbuffers = (VideoBuffer *)buffers;
            vbuffers[buf_idx].length = v4l_buf.length;
            vbuffers[buf_idx].offset = (size_t) v4l_buf.m.offset;
            vbuffers[buf_idx].start = (uint8_t *)mmap(NULL, v4l_buf.length,
                                      PROT_READ | PROT_WRITE, MAP_SHARED, camera_fd, v4l_buf.m.offset);

            if (vbuffers[buf_idx].start == MAP_FAILED)
            {
                int err = errno;
                ROS_ERROR("buffer map error: %d", err);
                return false;
            }

            if (ioctl(camera_fd, VIDIOC_QBUF, &v4l_buf) == -1)
            {
                ROS_ERROR("VIDIOC_QBUF error");
                return false;
            }
        }

        enum v4l2_buf_type type;
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (ioctl(camera_fd, VIDIOC_STREAMON, &type) == -1)
        {
            ROS_ERROR("VIDIOC_STREAMON error");
            return false;
        }

        camera_type = common::CameraInfo::Response::CAMERA_YUYV;
    }

    return true;
}

void close_mv_camera()
{
    if (camera_fd > 0)
    {
        CameraUnInit(camera_fd);
    }
}

void close_uvc_camera()
{
    if (camera_fd > 0)
    {
        enum v4l2_buf_type type;
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (ioctl(camera_fd, VIDIOC_STREAMOFF, &type))
        {
            ROS_ERROR("VIDIOC_STREAMOFF error");

            if (buffers != nullptr)
            {
                free(buffers);
                buffers = nullptr;
            }

            return;
        }

        VideoBuffer *vbuffers = (VideoBuffer *)buffers;

        for (buf_idx = 0; buf_idx < 4; buf_idx++)
        {
            munmap((void *)(vbuffers[buf_idx].start), vbuffers[buf_idx].length);
            vbuffers[buf_idx].start = nullptr;
        }

        if (buffers != nullptr)
        {
            free(buffers);
            buffers = nullptr;
        }
        ::close(camera_fd);
    }
}