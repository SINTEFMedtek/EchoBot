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

    private slots:
        void robotConnectSlot();
        void robotDisconnectSlot();
        void robotShutdownSlot();
        void cameraConnectSlot();
        void cameraDisconnectSlot();
        void usConnectSlot();
        void usDisconnectSlot();
        void usStreamerChangedSlot(const QString streamerOption);
        void updateCameraROI();

    private:
        int mWidgetWidth;
        int mWidgetHeight;

        SharedPointer<RobotInterface> mRobotInterface;
        SharedPointer<CameraInterface> mCameraInterface;
        SharedPointer<UltrasoundInterface> mUltrasoundInterface;

        QLineEdit *mRobotIPLineEdit, *mUsIPLineEdit, *mCameraMinDepthLineEdit,*mCameraMaxDepthLineEdit;
        QPushButton *mRobotConnectButton, *mRobotDisconnectButton, *mRobotShutdownButton;
        QPushButton *mCameraConnectButton, *mCameraDisconnectButton;
        QPushButton *mUSConnectButton, *mUSDisconnectButton;
        QComboBox *mUSStreamerOptionCBox;
        QLineEdit  *mCameraMinWidthLineEdit, *mCameraMaxWidthLineEdit, *mCameraMinHeightLineEdit, *mCameraMaxHeightLineEdit;

        QString mGraphicsFolderName;

        QWidget* getRobotConnectionWidget();
        QWidget* getCameraConnectionWidget();
        QWidget* getUltrasoundConnectionWidget();
};

}

#endif //ECHOBOT_CONNECTIONWIDGET_H
