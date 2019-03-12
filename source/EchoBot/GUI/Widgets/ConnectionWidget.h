#ifndef ECHOBOT_CONNECTIONWIDGET_H
#define ECHOBOT_CONNECTIONWIDGET_H

#include <QTabWidget>
#include <QLineEdit>
#include <QPushButton>

#include "EchoBot/Interfaces/RobotInterface.h"
#include "EchoBot/Interfaces/UltrasoundInterface.hpp"
#include "EchoBot/Interfaces/CameraInterface.hpp"

class ConnectionWidget : public QTabWidget
{
    Q_OBJECT

    public:
        ConnectionWidget(RobotInterfacePtr robotInterface, int widgetWidth=540);

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

    private:
        void setupWidget();
        void setupConnections();

        int mWidgetWidth;

        SharedPointer<RobotInterface> mRobotInterface;
        SharedPointer<CameraInterface> mCameraInterface;
        SharedPointer<UltrasoundInterface> mUltrasoundInterface;

        QLineEdit *mRobotIPLineEdit, *mUsIPLineEdit, *mCameraMinDepthLineEdit,*mCameraMaxDepthLineEdit;
        QPushButton *mRobotConnectButton, *mRobotDisconnectButton, *mRobotShutdownButton;
        QPushButton *mCameraConnectButton, *mCameraDisconnectButton;
        QPushButton *mUSConnectButton, *mUSDisconnectButton;
        QLineEdit  *mCameraMinWidthLineEdit, *mCameraMaxWidthLineEdit, *mCameraMinHeightLineEdit, *mCameraMaxHeightLineEdit;

        QString mGraphicsFolderName;

        QWidget* getRobotConnectionWidget();
        QWidget* getCameraConnectionWidget();
        QWidget* getUltrasoundConnectionWidget();
};

#endif //ECHOBOT_CONNECTIONWIDGET_H
