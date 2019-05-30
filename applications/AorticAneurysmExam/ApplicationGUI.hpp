#ifndef ECHOBOT_APPLICATIONGUI_H
#define ECHOBOT_APPLICATIONGUI_H

#include "EchoBot/Interfaces/RobotInterface.h"
#include "EchoBot/Interfaces/UltrasoundInterface.hpp"
#include "EchoBot/Interfaces/CameraInterface.hpp"
#include "EchoBot/Visualization/RobotVisualization.h"

#include "EchoBot/GUI/Widgets/RobotManualMoveTab.h"
#include "EchoBot/GUI/Widgets/ConnectionWidget.h"
#include "EchoBot/GUI/Widgets/RecordWidget.h"

#include "FAST/Visualization/Window.hpp"
#include "FAST/Streamers/Streamer.hpp"
#include "FAST/Streamers/FileStreamer.hpp"
#include "FAST/Streamers/ImageFileStreamer.hpp"
#include "FAST/Streamers/MeshFileStreamer.hpp"
#include "FAST/Streamers/IGTLinkStreamer.hpp"
#include "FAST/Streamers/ClariusStreamer.hpp"
#include "FAST/Tools/OpenIGTLinkClient/OpenIGTLinkClient.hpp"
#include "FAST/Visualization/LineRenderer/LineRenderer.hpp"

class QPushButton;
class QLabel;
class QTabWidget;
class QElapsedTimer;
class QListWidget;

namespace echobot {

class MouseListener;

class ApplicationGUI : public Window {
    FAST_OBJECT(ApplicationGUI)

private slots:
    void robotConnectButtonSlot();
    void robotDisconnectButtonSlot();
    void robotShutdownButtonSlot();
    void playbackButtonSlot(std::unordered_map<uint, Streamer::pointer> streamers);
    void stopPlaybackButtonSlot();

private:
    ApplicationGUI();

    SharedPointer<RobotInterface> mRobotInterface;
    SharedPointer<CameraInterface> mCameraInterface;
    SharedPointer<UltrasoundInterface> mUltrasoundInterface;
    SharedPointer<RealSenseStreamer> mCameraStreamer;
    SharedPointer<ClariusStreamer> mUltrasoundStreamer; // SharedPointer<IGTLinkStreamer> mUltrasoundStreamer;
    SharedPointer<RobotVisualizator> mRobotVisualizator;

    std::unordered_map<uint, Streamer::pointer> mCameraPlaybackStreamers;

    RobotManualMoveLayout* mMoveLayout;
    ConnectionWidget* mConnectionWidget;
    RecordWidget* mRecordWidget;

    void connectToCamera();
    void disconnectFromCamera();
    void stopStreaming();

    void connectToUltrasound();

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
    void setupCameraVisualization(bool cameraPlayback = false);
    void setupUltrasoundVisualization();

    void setupConnections();

    //void updateCameraROI();

    void calibrateSystem();
    void registerTarget();
    void moveToolToManualTarget();
    void moveToolToRegisteredTarget();
    void registerCloudToData();

    std::vector<Renderer::pointer> mView3DRenderers;
    std::vector<Renderer::pointer> mView2DRenderers;
    std::vector<Renderer::pointer> mViewUSRenderers;

    void clearRenderVectors();

    void updateRenderers(std::vector<Renderer::pointer> view3DRenderers,
                         std::vector<Renderer::pointer> view2DRenderers = std::vector<Renderer::pointer>(),
                         std::vector<Renderer::pointer> viewUSRenderers = std::vector<Renderer::pointer>());

    void loadPreoperativeData();
    Mesh::pointer mPreoperativeData;
};

}

#endif
