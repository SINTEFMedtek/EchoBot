#ifndef ECHOBOT_APPLICATIONGUI_H
#define ECHOBOT_APPLICATIONGUI_H

#include "EchoBot/Interfaces/Ultrasound/UltrasoundInterface.hpp"
#include "EchoBot/Interfaces/Camera/CameraInterface.hpp"
#include "EchoBot/Interfaces/Robot/RobotInterface.h"
#include "EchoBot/Visualization/RobotVisualization.h"

#include "EchoBot/GUI/Widgets/RobotManualMoveWidget.h"
#include "EchoBot/GUI/Widgets/ConnectionWidget.h"
#include "EchoBot/GUI/Widgets/RecordWidget.h"
#include "EchoBot/GUI/Widgets/CalibrationWidget.h"

#include "FAST/Visualization/Window.hpp"
#include "FAST/Streamers/Streamer.hpp"
#include "FAST/Streamers/FileStreamer.hpp"
#include "FAST/Streamers/ImageFileStreamer.hpp"
#include "FAST/Streamers/MeshFileStreamer.hpp"
#include "FAST/Streamers/OpenIGTLinkStreamer.hpp"
#include "FAST/Streamers/ClariusStreamer.hpp"
#include "FAST/Tools/OpenIGTLinkClient/OpenIGTLinkClient.hpp"
#include "FAST/Visualization/LineRenderer/LineRenderer.hpp"

class QPushButton;
class QTabWidget;

namespace echobot {

class MouseListener;

class ApplicationGUI : public Window {
    ECHOBOT_OBJECT(ApplicationGUI)

private slots:
    void robotConnectButtonSlot();
    void robotDisconnectButtonSlot();
    void robotShutdownButtonSlot();
    void playbackButtonSlot();
    void stopPlaybackButtonSlot();

private:
    ApplicationGUI();

    SharedPointer<RobotInterface> mRobotInterface;
    SharedPointer<CameraInterface> mCameraInterface;
    SharedPointer<UltrasoundInterface> mUltrasoundInterface;
    SharedPointer<RobotVisualizator> mRobotVisualizator;

    std::unordered_map<uint, Streamer::pointer> mCameraPlaybackStreamers;

    RobotManualMoveWidget* mRobotMoveWidget;
    ConnectionWidget* mConnectionWidget;
    RecordWidget* mRecordWidget;
    CalibrationWidget* mCalibrationWidget;

    void connectToCamera();
    void disconnectFromCamera();
    void stopStreaming();

    void connectToUltrasound();
    void disconnectFromUltrasound();

    void setupUI();

    QString mGraphicsFolderName;
    QPushButton *calibrateButton, *registerDataButton, *registerTargetButton, *moveToolManualButton;
    QPushButton *moveToolRegisteredButton, *planMoveToolRegisteredButton, *enableSegmentationButton;
    QTabWidget *tabWidget;

    void extractPointCloud();
    bool mCameraPlayback = false;
    bool mCameraStreaming = false;
    bool mUltrasoundStreaming = false;
    bool mTargetRegistered = false;
    bool mMovingToTarget = false;
    Vector6d mTargetJointConfig;
    Transform3d mTargetOpConfig;

    QWidget* getWorkflowWidget();

    void restartCamera();

    void setupRobotManipulatorVisualization();
    void setupCameraVisualization();
    void setupUltrasoundVisualization();

    void setupConnections();

    void calibrateSystem();
    void registerTarget();
    void moveToolToManualTarget();
    void planMoveToRegisteredTarget();
    void moveToolToRegisteredTarget();
    void registerCloudToData();
    void enableNeuralNetworkSegmentation();

    void loadPreoperativeData();
    Mesh::pointer mPreoperativeData;

    void addCoordinateAxis(Eigen::Affine3f transform);
    std::vector<Eigen::Affine3f> mCoordinateAxis;
    void renderCoordinateAxis(float axisLength = 500);

    void initializeRenderers();
};

}

#endif
