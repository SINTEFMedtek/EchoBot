#ifndef ECHOBOT_CONNECTIONWIDGET_H
#define ECHOBOT_CONNECTIONWIDGET_H

#include <QTabWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>

#include "EchoBot/Interfaces/Robot/RobotInterface.h"
#include "EchoBot/Interfaces/Ultrasound/UltrasoundInterface.hpp"
#include "EchoBot/Interfaces/Camera/CameraInterface.hpp"

namespace echobot
{

class ConnectionWidget : public QTabWidget
{
    Q_OBJECT

    public:
        ConnectionWidget(int widgetWidth=540, int widgetHeight=160);
        void addInterface(SensorInterface::pointer sensorInterface);

    signals:
        void robotConnected();
        void robotDisconnected();
        void robotShutdown();
        void cameraConnected();
        void cameraDisconnected();
        void usConnected();
        void usDisconnected();

    public slots:
        void updateCameraROI();

    private slots:
        void robotShutdownSlot();
        void usStreamerChangedSlot(const QString streamerOption);

        void robotToggleConnection();
        void usToggleConnection();
        void cameraToggleConnection();

    private:
        int mWidgetWidth;
        int mWidgetHeight;

        SharedPointer<RobotInterface> mRobotInterface;
        SharedPointer<CameraInterface> mCameraInterface;
        SharedPointer<UltrasoundInterface> mUltrasoundInterface;

        QLineEdit *mRobotIPLineEdit, *mUsIPLineEdit, *mCameraMinDepthLineEdit,*mCameraMaxDepthLineEdit;
        QPushButton *mCameraConnectionButton, *mUSConnectionButton, *mRobotConnectionButton, *mRobotShutdownButton;;
        QComboBox *mUSStreamerOptionCBox, *mRobotOptionCBox;
        QLineEdit  *mCameraMinWidthLineEdit, *mCameraMaxWidthLineEdit, *mCameraMinHeightLineEdit, *mCameraMaxHeightLineEdit;

        QString mGraphicsFolderName;
        bool mRobotConnected = false;
        bool mUSConnected = false;
        bool mCameraConnected = false;

        QWidget* createRobotConnectionWidget();
        QWidget* createCameraConnectionWidget();
        QWidget* createUltrasoundConnectionWidget();
};

}

#endif //ECHOBOT_CONNECTIONWIDGET_H
