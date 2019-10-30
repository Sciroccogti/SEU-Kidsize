#include "image_debuger.hpp"
#include <opencv2/opencv.hpp>
#include <bits/stdc++.h>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    ImageDebuger foo;
    foo.show();
    return app.exec();
}

ImageDebuger::ImageDebuger()
{
    height_ = 480;
    width_ = 640;

    srcLab = new ImageLabel(width_, height_);
    dstLab = new ImageLabel(width_, height_);
    curr_image_.create(height_, width_, CV_8UC3);
    curr_index_ = 0;
    infoLab = new QLabel("0/0");
    statusBar()->addWidget(infoLab);

    QHBoxLayout *imageLayout = new QHBoxLayout;
    imageLayout->addWidget(srcLab);
    imageLayout->addWidget(dstLab);

    funcBox = new QComboBox();
    QStringList funclist;
    funclist << "ball & goal";
    funclist << "field";
    funcBox->addItems(funclist);
    btnLoad = new QPushButton("Open Folder");
    btnLast = new QPushButton("Last Frame");
    btnNext = new QPushButton("Next Frame");
    boxAuto = new QCheckBox("Auto Play(ms)");
    boxAuto->setFixedWidth(120);
    delayEdit = new QLineEdit("1000");
    delayEdit->setFixedWidth(50);
    QHBoxLayout *ctrlLayout = new QHBoxLayout;
    ctrlLayout->addWidget(funcBox);
    ctrlLayout->addWidget(btnLoad);
    ctrlLayout->addWidget(btnLast);
    ctrlLayout->addWidget(btnNext);
    ctrlLayout->addWidget(boxAuto);
    ctrlLayout->addWidget(delayEdit);

    frmSlider = new QSlider(Qt::Horizontal);
    frmSlider->setEnabled(false);

    QVBoxLayout *mainLayout = new QVBoxLayout();
    mainLayout->addLayout(imageLayout);
    mainLayout->addLayout(ctrlLayout);
    mainLayout->addWidget(frmSlider);

    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);

    timer = new QTimer;
    connect(timer, &QTimer::timeout, this, &ImageDebuger::procTimer);
    connect(btnLoad, &QPushButton::clicked, this, &ImageDebuger::procBtnLoad);
    connect(btnLast, &QPushButton::clicked, this, &ImageDebuger::procBtnLast);
    connect(btnNext, &QPushButton::clicked, this, &ImageDebuger::procBtnNext);
    connect(boxAuto, &QCheckBox::stateChanged, this, &ImageDebuger::procBoxAuto);
    connect(frmSlider, &QSlider::valueChanged, this, &ImageDebuger::procFrmSlider);
    connect(srcLab, &ImageLabel::shot, this, &ImageDebuger::procShot);
    Reset();
}

void ImageDebuger::showSrc()
{
    Mat bgr = imread(String((curr_dir_+image_names_.at(curr_index_-1)).toStdString()));
    cvtColor(bgr, rgb_src_, CV_BGR2RGB);
    rgb_dst_ = rgb_src_.clone();
    cvtColor(bgr, hsv_src_, CV_BGR2HSV);
    QImage srcImage(rgb_src_.data, rgb_src_.cols, rgb_src_.rows, QImage::Format_RGB888);
    srcLab->set_image(srcImage);
}

void ImageDebuger::showDst()
{
    if(rgb_dst_.empty()) return;
    QImage dstImage(rgb_dst_.data, rgb_dst_.cols, rgb_dst_.rows, QImage::Format_RGB888);
    dstLab->set_image(dstImage);
}

void ImageDebuger::procImage(const unsigned int &index)
{
    if (index < 1 || index > image_names_.size())
    {
        return;
    }

    curr_index_ = index;
    infoLab->setText(QString::number(curr_index_) + "/" + QString::number(image_names_.size()));
    frmSlider->setValue(index);
    showSrc();

    if (funcBox->currentIndex() == 1) //ball and post detection
    {
        
        for(size_t i=0; i<rgb_dst_.rows; i++)
        {
            for(size_t j=0; j<rgb_dst_.cols; j++)
            {
                Vec3b tmp = hsv_src_.at<Vec3b>(i, j);
                if((tmp[0]>=H_low && tmp[0]<= H_high)
                    && (tmp[1]>=S_low && tmp[1]<= S_high)
                    && (tmp[2]>=V_low && tmp[2]<= V_high))
                {
                    rgb_dst_.at<Vec3b>(i, j) = Vec3b(0, 255, 0);
                }
            }
        }
        
        for(size_t j=0; j<rgb_dst_.cols; j++)
        {
            size_t row = rgb_dst_.rows-1;
            for(int i=rgb_dst_.rows-1; i>=0; i--)
            {
                Vec3b tmp = hsv_src_.at<Vec3b>(i, j);
                if((tmp[0]>=H_low && tmp[0]<= H_high)
                    && (tmp[1]>=S_low && tmp[1]<= S_high)
                    && (tmp[2]>=V_low && tmp[2]<= V_high))
                {
                    row = i;
                }
            }
            rgb_dst_.at<Vec3b>(row, j) = Vec3b(255, 0, 0);
        }
        showDst();
    }
}

void ImageDebuger::procBtnLast()
{
    curr_index_--;

    if (curr_index_ < 1)
    {
        curr_index_ = image_names_.size();
    }

    procImage(curr_index_);
}

void ImageDebuger::procBtnNext()
{
    curr_index_++;

    if (curr_index_ > image_names_.size())
    {
        curr_index_ = 1;
    }

    procImage(curr_index_);
}

void ImageDebuger::procBtnLoad()
{
    timer->stop();
    curr_dir_ = QFileDialog::getExistingDirectory(this, "Open image directory", QDir::homePath())+"/";
    if (curr_dir_.isEmpty())
    {
        return;
    }

    QDir dir(curr_dir_);
    QStringList nameFilters;
    nameFilters << "*.jpg" << "*.png";
    image_names_.clear();
    image_names_ = dir.entryList(nameFilters, QDir::Files|QDir::Readable, QDir::Name);
    if (!image_names_.empty())
    {
        frmSlider->setEnabled(true);
        frmSlider->setMinimum(1);
        frmSlider->setMaximum(image_names_.size());
        procImage(1);
    }
}

void ImageDebuger::procBoxAuto()
{
    if (boxAuto->checkState() == Qt::Checked)
    {
        int delay = delayEdit->text().toInt();

        if (delay < 10)
        {
            delay = 10;
        }

        timer->start(delay);
    }
    else
    {
        timer->stop();
    }
}

void ImageDebuger::procShot(QRect rect)
{
    if(rgb_src_.empty()) return;
    if (rect.width() > 10 && rect.height() > 10)
    {
        int x, y, w, h;
        x = rect.left();
        y = rect.top();
        w = rect.width();
        h = rect.height();
        
        if (x + w < width_ && y + h < height_)
        {
            if (funcBox->currentIndex() == 1) // sampling
            {
                for(int i=y; i<=y+h; i++)
                {
                    for(int j=x; j<=x+w; j++)
                    {
                        Vec3b tmp = hsv_src_.at<Vec3b>(i, j);
                        H.push_back(tmp[0]);
                        S.push_back(tmp[1]);
                        V.push_back(tmp[2]);
                    }
                }
                sort(H.begin(), H.end());
                sort(S.begin(), S.end());
                sort(V.begin(), V.end());
                size_t n = H.size();
                size_t o = n*0.05;
                vector<uint8_t> NH,NS,NV;
                for(int i=o; i<n-o; i++)
                {
                    NH.push_back(H[i]);
                    NS.push_back(S[i]);
                    NV.push_back(V[i]);
                }
                uint8_t MH=Mean(NH), MS=Mean(NS), MV=Mean(NV);
                float SH=0.0, SS=0.0, SV=0.0;
                size_t nn = NH.size();
                for(size_t i=0; i<nn; i++)
                {
                    SH += pow(NH[i]-MH, 2);
                    SS += pow(NS[i]-MS, 2);
                    SV += pow(NV[i]-MV, 2);
                }
                SH = sqrt(SH/nn);
                SS = sqrt(SS/nn);
                SV = sqrt(SV/nn);
                H_low = MH-3*SH; H_high = MH+3*SH;
                S_low = MS-3*SS; S_high = MS+3*SS;
                V_low = MV-3*SV; V_high = MV+3*SV;
                procImage(curr_index_);
            }
        }
    }
    else{
        Reset();
    }
}

void ImageDebuger::Reset()
{
    H_low = 255; H_high = 255;
    S_low = 255; S_high = 255;
    V_low = 255; V_high = 255;
    H.clear();
    S.clear();
    V.clear();
}

void ImageDebuger::procFrmSlider(int v)
{
    procImage(v);
}


void ImageDebuger::procTimer()
{
    procBtnNext();
}
