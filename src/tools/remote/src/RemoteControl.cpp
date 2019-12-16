#include "RemoteControl.hpp"

using namespace std;

const int range_ = 200;
const float scale_d = 10.0;
const float scale_xy = 5000.0;

Slider::Slider(Qt::Orientation orientation, QString name, float s, QWidget *parent)
    : QWidget(parent)
{
    scale = s;
    valueLabel = new QLabel();
    if(orientation == Qt::Horizontal)
    {
        QHBoxLayout *layout = new QHBoxLayout();
        slider = new QSlider(Qt::Horizontal);
        layout->addWidget(new QLabel(name));
        layout->addWidget(slider);
        layout->addWidget(valueLabel);
        setLayout(layout);
    }
    else
    {
        QVBoxLayout *layout = new QVBoxLayout();
        slider = new QSlider(Qt::Vertical);
        layout->addWidget(valueLabel);
        layout->addWidget(slider);
        layout->addWidget(new QLabel(name));
        setLayout(layout);
    }
    slider->setRange(-200, 200);
    connect(slider, &QSlider::valueChanged, this, &Slider::procValueChanged);
}

void Slider::setRange(int mini, int maxi)
{
    slider->setRange(mini, maxi);
}

void Slider::procValueChanged(int v)
{
    valueLabel->setText(QString::number(v/scale));
    emit changed(v);
}

RemoteControl::RemoteControl(ros::NodeHandle &node): mNode(node)
{
    QGroupBox *walkBox = new QGroupBox("Walk Control");
    QVBoxLayout *walkLayout = new QVBoxLayout();
    xSlider = new Slider(Qt::Horizontal, "X", scale_xy);
    xSlider->setMinimumWidth(200);
    ySlider = new Slider(Qt::Horizontal, "Y", scale_xy);
    ySlider->setMinimumWidth(200);
    dirSlider = new Slider(Qt::Horizontal, "Dir", scale_d);
    dirSlider->setMinimumWidth(200);
    startCheck = new QCheckBox("Walk");
    walkLayout->addWidget(xSlider);
    walkLayout->addWidget(ySlider);
    walkLayout->addWidget(dirSlider);
    walkLayout->addWidget(startCheck);
    walkBox->setLayout(walkLayout);

    QGroupBox *headBox = new QGroupBox("Head Control");
    QVBoxLayout *headLayout = new QVBoxLayout();
    yawSlider = new Slider(Qt::Horizontal, "Yaw");
    yawSlider->setRange(-120, 120);
    pitchSlider = new Slider(Qt::Horizontal, "Pitch");
    pitchSlider->setRange(-90, 90);
    headLayout->addWidget(yawSlider);
    headLayout->addWidget(pitchSlider);
    headBox->setLayout(headLayout);

    QVBoxLayout *leftLayout = new QVBoxLayout();
    leftLayout->addWidget(walkBox);
    leftLayout->addWidget(headBox);

    QVBoxLayout *rightLayout = new QVBoxLayout();
    rightLayout->addWidget(new QLabel("Act Control"));
    actsWidget = new QListWidget();
    QStringList acts;
    acts<<"back_getup"<<"front_getup"<<"left_kick"<<"right_kick"<<"ready"<<"reset";
    actsWidget->addItems(acts);
    rightLayout->addWidget(actsWidget);
    rightLayout->addWidget(new QPushButton("Run Action"));

    QHBoxLayout *mainLayout = new QHBoxLayout();
    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(rightLayout);

    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);
    this->setWindowTitle("RemoteControl");
}

