#ifndef FAST_KINECT_TRACKING_GUI_HPP_
#define FAST_KINECT_TRACKING_GUI_HPP_

#include "FAST/Visualization/MultiViewWindow.hpp"
#include "visualization/robotVisualization.h"
#include "widgets/robotManualMoveTab.h"
#include "RobotInterface.h"

class QPushButton;
class QLabel;
class QTabWidget;

namespace fast {

class KinectStreamer;
class KinectTracking;

class ApplicationGUI : public MultiViewWindow {
    FAST_OBJECT(ApplicationGUI)

private slots:
    void robotConnectButtonSlot();
    void robotDisconnectButtonSlot();
    void robotShutdownButtonSlot();

private:
    ApplicationGUI();
    RobotInterfacePtr mRobotInterface;

    SharedPointer<KinectStreamer> mStreamer;
    SharedPointer<KinectTracking> mTracking;

    RobotManualMoveLayout* mMoveLayout;

    void connectToCamera();
    void disconnectFromCamera();

    void setupUI();

    QString mGraphicsFolderName;
    QLineEdit *robotIPLineEdit;
    QPushButton *robotConnectButton, *robotDisconnectButton,*robotShutdownButton;
    QPushButton *cameraConnectButton, *cameraDisconnectButton;
    QTabWidget *tabWidget;

    void setRobotConnectionLayout(QVBoxLayout *parent);
    void setCameraConnectionLayout(QVBoxLayout *parent);

    void setupRobotToolVisualization();
    void setupRobotManipulatorVisualization();

    void setupConnections();

};

}

#endif
