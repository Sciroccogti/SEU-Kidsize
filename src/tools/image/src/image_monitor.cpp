#include <QApplication>
#include "image_monitor.hpp"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <common/HeadTask.h>
#include <common/common.hpp>

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

    pitchSlider = new QSlider(Qt::Vertical);
    pitchSlider->setRange(-90, 90);
    yawSlider = new QSlider(Qt::Horizontal);
    yawSlider->setRange(-110, 110);

    imageSrcBox = new QCheckBox("Send Src");
    imageSaveBox = new QCheckBox("Save Image");

    QVBoxLayout *leftLayout = new QVBoxLayout();
    leftLayout->addWidget(imageLab);
    leftLayout->addWidget(yawSlider);

    QVBoxLayout *ctrlLayout = new QVBoxLayout;
    ctrlLayout->addWidget(imageSrcBox);
    ctrlLayout->addWidget(imageSaveBox);

    QHBoxLayout *mainLayout = new QHBoxLayout();
    mainLayout->addLayout(leftLayout);
    mainLayout->addWidget(pitchSlider);
    mainLayout->addLayout(ctrlLayout);

    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);

    yawLab = new QLabel();
    pitchLab = new QLabel();
    netLab = new QLabel();
    netLab->setFixedWidth(100);

    statusBar()->addWidget(pitchLab);
    statusBar()->addWidget(yawLab);
    statusBar()->addWidget(netLab);
    timer = new QTimer;
    timer->start(50);
    image_count_=0;

    connect(timer, &QTimer::timeout, this, &ImageMonitor::procTimer);
    connect(yawSlider, &QSlider::valueChanged, this, &ImageMonitor::procSlider);
    connect(pitchSlider, &QSlider::valueChanged, this, &ImageMonitor::procSlider);
    connect(imageLab, &ImageLabel::shot, this, &ImageMonitor::procShot);
    connect(imageSrcBox, &QCheckBox::stateChanged, this, &ImageMonitor::procImageBox);
    headPub = node.advertise<common::HeadTask>("/headtask", 2);
    imageSub = node.subscribe("/vision/image/compressed", 2, &ImageMonitor::ImageUpdate, this);
}

void ImageMonitor::ImageUpdate(const sensor_msgs::CompressedImage::ConstPtr &image)
{
    vector<uint8_t> buf(image->data.size());
    memcpy(&(buf[0]), &(image->data[0]), image->data.size());
    try
    {
        Mat image_data = imdecode(buf, cv::IMREAD_COLOR);
        if(imageSaveBox->isChecked()&&image_count_++%10==0)
        {
            imwrite(String(get_time()+".png"), image_data);
        }
        Mat dst;
        cvtColor(image_data, dst, CV_BGR2RGB);
        QImage dstImage((const unsigned char *)(dst.data), dst.cols, dst.rows, QImage::Format_RGB888);
        imageLab->set_image(dstImage);
    }
    catch(std::exception &e){
        ROS_INFO("%s", e.what());
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

void ImageMonitor::procImageBox(int state)
{
    std_srvs::SetBool srv;
    srv.request.data = state;
    ros::service::call("/sendsrc", srv);
}

void ImageMonitor::procSlider(int v)
{
    pitchLab->setText(QString::number(pitchSlider->value()));
    yawLab->setText(QString::number(yawSlider->value()));
    float yaw = -(float)(yawSlider->value());
    float pitch = -(float)(pitchSlider->value());
    common::HeadTask task;
    task.mode = task.ModeLookAt;
    task.yaw = yaw;
    task.pitch = pitch;
    headPub.publish(task);
}
