#ifndef FAST_KINECT_TRACKING_GUI_HPP_
#define FAST_KINECT_TRACKING_GUI_HPP_

#include "FAST/Visualization/MultiViewWindow.hpp"
#include "FAST/Streamers/IGTLinkStreamer.hpp"
#include "visualization/robotVisualization.h"
#include "FAST/Tools/OpenIGTLinkClient/OpenIGTLinkClient.hpp"

#include "widgets/robotManualMoveTab.h"
#include "RobotInterface.h"

class QPushButton;
class QLabel;
class QTabWidget;
class QElapsedTimer;
class QListWidget;

namespace fast {

class KinectStreamer;
class CameraInterface;

class ApplicationGUI : public MultiViewWindow {
    FAST_OBJECT(ApplicationGUI)

private slots:
    void robotConnectButtonSlot();
    void robotDisconnectButtonSlot();
    void robotShutdownButtonSlot();

private:
    ApplicationGUI();

    SharedPointer<RobotInterface> mRobotInterface;
    SharedPointer<CameraInterface> mCameraInterface;
    SharedPointer<KinectStreamer> mCameraStreamer;
    SharedPointer<IGTLinkStreamer> mUltrasoundStreamer;

    RobotManualMoveLayout* mMoveLayout;

    void connectToCamera();
    void disconnectFromCamera();

    void connectToUltrasound();

    void setupUI();

    QString mGraphicsFolderName;
    QLineEdit *robotIPLineEdit, *cameraMinDepthLineEdit,*cameraMaxDepthLineEdit, *usIPLineEdit;
    QPushButton *robotConnectButton, *robotDisconnectButton, *robotShutdownButton;
    QPushButton *cameraConnectButton, *cameraDisconnectButton;
    QPushButton *usConnectButton, *usDisconnectButton;
    QTabWidget *tabWidget;

    QPushButton *mRecordButton, *mPlayButton;
    QLineEdit* mStorageDir, *mRecordingNameLineEdit;
    QLabel* mRecordingInformation;
    QElapsedTimer* mRecordTimer;
    QListWidget* mRecordingsList;
    std::string mRecordingName;


    void playRecording();
    void extractPointCloud();
    bool mRecording = false;
    bool mPlaying = false;

    void setRobotConnectionLayout(QVBoxLayout *parent);
    void setCameraConnectionLayout(QVBoxLayout *parent);
    void setUltrasoundConnectionLayout(QVBoxLayout *parent);
    void setRecordingLayout(QVBoxLayout *parent);
    void restartCamera();

    void setupRobotToolVisualization();
    void setupRobotManipulatorVisualization();

    void setupConnections();

    void updateCameraROI();
    void refreshRecordingsList();
    void toggleRecord();
};

}

#endif
