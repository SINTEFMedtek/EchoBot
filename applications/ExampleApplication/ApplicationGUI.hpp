#ifndef ECHOBOT_APPLICATIONGUI_H
#define ECHOBOT_APPLICATIONGUI_H

#include "EchoBot/Interfaces/Ultrasound/UltrasoundInterface.hpp"
#include "EchoBot/Interfaces/Camera/CameraInterface.hpp"
#include "EchoBot/Interfaces/Robot/RobotInterface.h"
#include "EchoBot/Visualization/RobotVisualization.h"

#include "EchoBot/GUI/Widgets/RobotManualMoveWidget.h"
#include "EchoBot/GUI/Widgets/ConnectionWidget.h"
#include "EchoBot/GUI/Widgets/RecordWidget.h"

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
    FAST_OBJECT(ApplicationGUI)

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

    void connectToCamera();
    void disconnectFromCamera();
    void stopStreaming();

    void connectToUltrasound();
    void disconnectFromUltrasound();

    void setupUI();

    QString mGraphicsFolderName;
    QPushButton *calibrateButton, *registerDataButton, *registerTargetButton, *moveToolManualButton, *moveToolRegisteredButton;
    QTabWidget *tabWidget;

    void extractPointCloud();
    bool mCameraPlayback = false;
    bool mCameraStreaming = false;
    bool mUltrasoundStreaming = false;
    bool mTargetRegistered = false;
    bool mMovingToTarget = false;

    QWidget* getWorkflowWidget();

    void restartCamera();

    void setupRobotManipulatorVisualization();
    void setupCameraVisualization();
    void setupUltrasoundVisualization();

    void setupConnections();

    void calibrateSystem();
    void registerTarget();
    void moveToolToManualTarget();
    void moveToolToRegisteredTarget();
    void registerCloudToData();

    void loadPreoperativeData();
    Mesh::pointer mPreoperativeData;

    void initializeRenderers();
};

}

#endif
