#ifndef __IMAGE_DEBUGER_HPP
#define __IMAGE_DEBUGER_HPP

#include <bits/stdc++.h>
#include <QtWidgets>
#include "ImageLabel.hpp"
#include <opencv2/opencv.hpp>

class ImageDebuger: public QMainWindow
{
    Q_OBJECT
public:
    ImageDebuger();
public slots:
    void procTimer();
    void procBtnLoad();
    void procBtnLast();
    void procBtnNext();
    void procBoxAuto();
    void procShot(QRect rect);
    void procFrmSlider(int v);
    
private:
    void procImage(const unsigned int &index);
    void showSrc();
    void showDst();
    void Reset();
    
    template<typename T>
    T Mean(std::vector<T> &data)
    {
        int sum=0;
        for(size_t i=0; i<data.size(); i++)
            sum+=data[i];
        return static_cast<T>(sum/data.size());
    }
    
    std::vector<uint8_t> H, S, V;
    uint8_t H_low, H_high, S_low, S_high, V_low, V_high;

    QPushButton *btnLoad, *btnNext, *btnLast;
    QCheckBox *boxAuto;
    ImageLabel *srcLab, *dstLab;
    QLabel *infoLab;
    QTimer *timer;
    QSlider *frmSlider;
    QLineEdit *delayEdit;
    QComboBox *funcBox;
    unsigned int curr_index_;
    cv::Mat curr_image_;
    QString curr_dir_;
    QStringList image_names_;
    cv::Mat rgb_src_, hsv_src_, rgb_dst_;
    int width_;
    int height_;
};

#endif