#ifndef ECHOBOT_APPLICATIONGUI_H
#define ECHOBOT_APPLICATIONGUI_H

#include "FAST/Visualization/Window.hpp"
#include "FAST/Streamers/Streamer.hpp"
#include "FAST/Streamers/FileStreamer.hpp"
#include "FAST/Streamers/ImageFileStreamer.hpp"
#include "FAST/Streamers/MeshFileStreamer.hpp"
#include "FAST/Streamers/IGTLinkStreamer.hpp"
#include "FAST/Tools/OpenIGTLinkClient/OpenIGTLinkClient.hpp"
#include "FAST/Visualization/LineRenderer/LineRenderer.hpp"

#include "EchoBot/Interfaces/RobotInterface.h"
#include "EchoBot/Interfaces/UltrasoundInterface.hpp"
#include "EchoBot/Interfaces/CameraInterface.hpp"
#include "EchoBot/Visualization/RobotVisualization.h"
#include "EchoBot/GUI/Widgets/RobotManualMoveTab.h"

class QPushButton;
class QLabel;
class QTabWidget;
class QElapsedTimer;
class QListWidget;

namespace fast {

class RealSenseStreamer;
class MouseListener;

class ApplicationGUI : public Window {
    FAST_OBJECT(ApplicationGUI)

private slots:
    void robotConnectButtonSlot();
    void robotDisconnectButtonSlot();
    void robotShutdownButtonSlot();

private:
    ApplicationGUI();

    SharedPointer<RobotInterface> mRobotInterface;
    SharedPointer<CameraInterface> mCameraInterface;
    SharedPointer<UltrasoundInterface> mUltrasoundInterface;

    SharedPointer<RealSenseStreamer> mCameraStreamer;
    std::unordered_map<uint, Streamer::pointer> mCameraPlaybackStreamers;

    SharedPointer<IGTLinkStreamer> mUltrasoundStreamer;

    RobotVisualizator *mRobotVisualizator;
    RobotManualMoveLayout* mMoveLayout;

    void connectToCamera();
    void disconnectFromCamera();
    void stopStreaming();

    void connectToUltrasound();

    void setupUI();

    QString mGraphicsFolderName;
    QLineEdit *mRobotIPLineEdit, *mUsIPLineEdit, *mCameraMinDepthLineEdit,*mCameraMaxDepthLineEdit;
    QLineEdit  *mCameraMinWidthLineEdit, *mCameraMaxWidthLineEdit, *mCameraMinHeightLineEdit, *mCameraMaxHeightLineEdit;
    QPushButton *robotConnectButton, *robotDisconnectButton, *robotShutdownButton;
    QPushButton *cameraConnectButton, *cameraDisconnectButton;
    QPushButton *usConnectButton, *usDisconnectButton;
    QPushButton *calibrateButton, *registerDataButton, *registerTargetButton, *moveToolManualButton, *moveToolRegisteredButton;
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
    bool mCameraPlayback = false;
    bool mCameraStreaming = false;
    bool mUltrasoundStreaming = false;
    bool mTargetRegistered = false;
    bool mMovingToTarget = false;

    QWidget* getRobotConnectionWidget();
    QWidget* getCameraConnectionWidget();
    QWidget* getUltrasoundConnectionWidget();
    QWidget* getRecordingWidget();
    QWidget* getWorkflowWidget();

    void restartCamera();

    void setupRobotManipulatorVisualization();
    void setupCameraVisualization();
    void setupUltrasoundVisualization();

    void setupConnections();

    void updateCameraROI();
    void refreshRecordingsList();
    void toggleRecord();

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


    static LineRenderer::pointer createCoordinateFrameRenderer(Eigen::Affine3f transform);
};

}

#endif
