#ifndef __IMAGE_MONITOR_HPP
#define __IMAGE_MONITOR_HPP

#include <QtWidgets>
#include <atomic>
#include "ImageLabel.hpp"
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>

class ImageMonitor: public QMainWindow
{
    Q_OBJECT
public:
    ImageMonitor(ros::NodeHandle &node);
    void ImageUpdate(const sensor_msgs::CompressedImage::ConstPtr &image);
public slots:
    void procSlider(int v);
    void procShot(QRect rect);
    void procImageBox(int state);
    void procTimer();

private:
    ImageLabel *imageLab;
    QLabel *yawLab, *pitchLab, *netLab;
    QSlider *pitchSlider, *yawSlider;
    QCheckBox *imageSrcBox;
    QCheckBox *imageSaveBox;
    QTimer *timer;

    cv::Mat curr_image_;
    int width_;
    int height_;
    unsigned int image_count_;
    ros::NodeHandle &node;
    ros::Publisher headPub;
    ros::Subscriber imageSub;
};

#endif
