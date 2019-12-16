#ifndef __IMAGE_MONITOR_HPP
#define __IMAGE_MONITOR_HPP

#include <QtWidgets>
#include <atomic>
#include "ImageLabel.hpp"
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

class ImageMonitor: public QMainWindow
{
    Q_OBJECT
public:
    ImageMonitor(ros::NodeHandle &node);
    void ImageUpdate(const sensor_msgs::Image::ConstPtr &image);
public slots:
    void procShot(QRect rect);
    void procSendTypeBox(int idx);
    void procTimer();

private:
    ImageLabel *imageLab;
    QComboBox *sendTypeBox;
    QCheckBox *imageSaveBox;
    QTimer *timer;

    cv::Mat curr_image_;
    int width_;
    int height_;
    unsigned int image_count_;
    ros::NodeHandle &node;
    image_transport::Subscriber imageSub;
};

#endif
