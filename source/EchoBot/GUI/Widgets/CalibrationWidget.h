#ifndef ECHOBOT_CALIBRATIONWIDGET_H
#define ECHOBOT_CALIBRATIONWIDGET_H

#include <QTabWidget>
#include <QPushButton>
#include <QComboBox>
#include <QCheckBox>

#include "EchoBot/Interfaces/Robot/RobotInterface.h"
#include "EchoBot/Interfaces/Camera/CameraInterface.hpp"
#include "EchoBot/Interfaces/Ultrasound/UltrasoundInterface.hpp"
#include "EchoBot/Utilities/CalibrationTool.h"

class QPushButton;

namespace echobot
{

class CalibrationWidget : public QTabWidget
{
    Q_OBJECT

    public:
        CalibrationWidget(int widgetWidth=540, int widgetHeight=220);
        void addInterface(SensorInterface::pointer sensorInterface);

        void calibrateSystem();

    private:
        void setupWidget();
        void setupConnections();

        int mWidgetWidth;
        int mWidgetHeight;

        SharedPointer<RobotInterface> mRobotInterface;
        SharedPointer<CameraInterface> mCameraInterface;
        SharedPointer<UltrasoundInterface> mUltrasoundInterface;
        SharedPointer<CalibrationTool> mCalibrationTool;

        QPushButton* mCalibrateButton;

        QWidget* getCalibrationWidget();

        void updateToolToUSTransform();
};

} // end namespace echobot

#endif //ECHOBOT_CALIBRATIONWIDGET_H
