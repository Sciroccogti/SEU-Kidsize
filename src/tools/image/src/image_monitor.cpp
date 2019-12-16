#include <QApplication>
#include "image_monitor.hpp"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <ros/ros.h>
#include <common/common.hpp>
#include <common/SetInt.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_monitor");
    ros::NodeHandle node;
    QApplication app(argc, argv);
    ImageMonitor foo(node);
    foo.show();
    return app.exec();
}

ImageMonitor::ImageMonitor(ros::NodeHandle &n): node(n)
{
    height_ = 480;
    width_ = 640;

    imageLab = new ImageLabel(width_, height_);
    curr_image_.create(height_, width_, CV_8UC3);

    sendTypeBox = new QComboBox();
    QStringList tps;
    tps<<"None"<<"Source"<<"Target";
    sendTypeBox->addItems(tps);
    imageSaveBox = new QCheckBox("Save Image");

    QVBoxLayout *leftLayout = new QVBoxLayout();
    leftLayout->addWidget(imageLab);

    QVBoxLayout *ctrlLayout = new QVBoxLayout;
    ctrlLayout->addWidget(sendTypeBox);
    ctrlLayout->addWidget(imageSaveBox);

    QHBoxLayout *mainLayout = new QHBoxLayout();
    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(ctrlLayout);

    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);

    timer = new QTimer;
    timer->start(50);
    image_count_=0;

    connect(timer, &QTimer::timeout, this, &ImageMonitor::procTimer);
    connect(imageLab, &ImageLabel::shot, this, &ImageMonitor::procShot);
    typedef void (QComboBox::*QComboIntSignal)(int);
    connect(sendTypeBox, static_cast<QComboIntSignal>(&QComboBox::activated), 
                            this, &ImageMonitor::procSendTypeBox);
    image_transport::ImageTransport it(node);
    image_transport::TransportHints hints("compressed");
    imageSub = it.subscribe("/result/vision", 1, &ImageMonitor::ImageUpdate, this, hints);
}

void ImageMonitor::ImageUpdate(const sensor_msgs::Image::ConstPtr &msg)
{
    try
    {
        // First let cv_bridge do its magic
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        curr_image_ = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        try
        {
            // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
            if (msg->encoding == "CV_8UC3")
            {
                // assuming it is rgb
                curr_image_ = cv_ptr->image;
            } else if (msg->encoding == "8UC1") {
                // convert gray to rgb
                cv::cvtColor(cv_ptr->image, curr_image_, CV_GRAY2RGB);
            } else if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
                imageLab->set_image(QImage());
            } else {
                qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
                imageLab->set_image(QImage());
                return;
            }
        }
        catch (cv_bridge::Exception& e)
        {
            qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
            imageLab->set_image(QImage());
            return;
        }
    }
    QImage dstImage(curr_image_.data, curr_image_.cols, curr_image_.rows, QImage::Format_RGB888);
    imageLab->set_image(dstImage);
    image_count_ ++;
    if(imageSaveBox->isChecked() && image_count_%10 == 0)
    {
        Mat bgr;
        cvtColor(curr_image_, bgr, CV_RGB2BGR);
        string name = get_current_time() + ".png";
        imwrite(name, bgr);
    }
}

void ImageMonitor::procTimer()
{
    ros::spinOnce();
}

void ImageMonitor::procShot(QRect rect)
{
    if (rect.width() > 5 && rect.height() > 5)
    {
        int x, y, w, h;
        x = rect.left();
        y = rect.top();
        w = rect.width();
        h = rect.height();
    }
}

void ImageMonitor::procSendTypeBox(int idx)
{
    common::SetInt srv;
    srv.request.number = idx;
    ros::service::call("/setting/sendtype", srv);
}
