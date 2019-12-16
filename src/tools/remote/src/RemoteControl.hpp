#ifndef __IMAGE_DEBUGER_HPP
#define __IMAGE_DEBUGER_HPP

#include <bits/stdc++.h>
#include <QtWidgets>
#include <ros/ros.h>

class Slider: public QWidget
{
    Q_OBJECT 
public:
    Slider(Qt::Orientation orientation, QString name, float s=1, QWidget *parent = Q_NULLPTR);
    void setRange(int mini, int maxi);
    int value() const
    {
        return slider->value();
    }
public slots:
    void procValueChanged(int v);

signals:
    void changed(int v);

private:
    QLabel *valueLabel;
    QSlider *slider;
    float scale;
};

class RemoteControl : public QMainWindow
{
    Q_OBJECT
public:
    RemoteControl(ros::NodeHandle &node);

private:
    QCheckBox *startCheck;
    Slider *dirSlider, *xSlider, *ySlider, *yawSlider, *pitchSlider;
    QListWidget *actsWidget;
    QComboBox *stBox;
    ros::NodeHandle &mNode;
    QTimer *timer;
};


#endif
