//
// Created by androst on 14.02.19.
//

#include <QGridLayout>
#include <QGroupBox>
#include <QTimer>

#include "CalibrationWidget.h"


namespace echobot
{

CalibrationWidget::CalibrationWidget(int widgetWidth, int widgetHeight) :
    mWidgetWidth(widgetWidth),
    mWidgetHeight(widgetHeight)
{
    setupWidget();
    setupConnections();

    this->setFixedWidth(mWidgetWidth);
    this->setFixedHeight(mWidgetHeight);

    mCalibrationTool = CalibrationTool::New();
}

void CalibrationWidget::setupWidget()
{
    QWidget *recordWidget = getCalibrationWidget();
    this->addTab(recordWidget, "Calibration");
}

void CalibrationWidget::setupConnections()
{
    QObject::connect(mCalibrateButton, &QPushButton::clicked, std::bind(&CalibrationWidget::calibrateSystem, this));
}

QWidget* CalibrationWidget::getCalibrationWidget()
{
    QGroupBox* group = new QGroupBox();
    group->setFlat(true);

    QGridLayout *mainLayout = new QGridLayout();
    group->setLayout(mainLayout);

    int row = 0;
    mCalibrateButton = new QPushButton();
    mainLayout->addWidget(mCalibrateButton,row,0,1,2);
    mCalibrateButton->setText("Calibrate");
    mCalibrateButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    return group;
}

void CalibrationWidget::calibrateSystem() {
    mCalibrationTool->calibrate();

    mRobotInterface->getRobot()->getCoordinateSystem()->set_rMb(mCalibrationTool->get_rMb());
    mRobotInterface->getRobot()->getCoordinateSystem()->set_eeMt(mCalibrationTool->get_eeMt());

    QTimer *timer = new QTimer(this);
    QObject::connect(timer, &QTimer::timeout, std::bind(&CalibrationWidget::updateToolToUSTransform, this));
    timer->start(25);
}

void CalibrationWidget::addInterface(SensorInterface::pointer sensorInterface)
{
    if(sensorInterface->getNameOfClass() == "RobotInterface"){
        mRobotInterface = std::dynamic_pointer_cast<RobotInterface>(sensorInterface);
    } else if (sensorInterface->getNameOfClass() == "CameraInterface"){
        mCameraInterface = std::dynamic_pointer_cast<CameraInterface>(sensorInterface);
    } else if (sensorInterface->getNameOfClass() == "UltrasoundInterface")
    {
        mUltrasoundInterface = std::dynamic_pointer_cast<UltrasoundInterface>(sensorInterface);
    }
}

void CalibrationWidget::updateToolToUSTransform() {
    auto currentImage = mUltrasoundInterface->getProcessObject()->getProcessedImage();
    auto sagitta = 45*(1-cos(0.637045175)); // Sagitta Clarius (45 mm ROC + 73 deg FOV)

    Eigen::Affine3d offset = Eigen::Affine3d::Identity();
    Eigen::Vector3d translation(-(double)(currentImage->getWidth()*currentImage->getSpacing()(0)/2), sagitta, 0);

    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())*
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());

    offset.translate(translation);
    offset.linear() = offset.linear()*m;

    Eigen::Affine3d rMb = mRobotInterface->getRobot()->getCoordinateSystem()->get_rMb();
    Eigen::Affine3d eeMt = mRobotInterface->getRobot()->getCoordinateSystem()->get_eeMt();
    Eigen::Affine3d bMee = mRobotInterface->getCurrentState()->getTransformToJoint(6);
    Eigen::Affine3d transform = rMb*bMee*eeMt*offset;

    mUltrasoundInterface->setImageTransform(transform);
}

}