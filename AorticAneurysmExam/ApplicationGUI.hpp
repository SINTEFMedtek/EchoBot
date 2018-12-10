#ifndef FASTROMO_APPLICATIONGUI_H
#define FASTROMO_APPLICATIONGUI_H

#include "FAST/Visualization/Window.hpp"
#include "FAST/Streamers/IGTLinkStreamer.hpp"
#include "FAST/Tools/OpenIGTLinkClient/OpenIGTLinkClient.hpp"

#include "RobotInterface.h"
#include "UltrasoundInterface.hpp"
#include "CameraInterface.hpp"
#include "visualization/robotVisualization.h"
#include "widgets/robotManualMoveTab.h"

class QPushButton;
class QLabel;
class QTabWidget;
class QElapsedTimer;
class QListWidget;

namespace fast {

class KinectStreamer;

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
    SharedPointer<KinectStreamer> mCameraStreamer;
    SharedPointer<IGTLinkStreamer> mUltrasoundStreamer;
    SharedPointer<UltrasoundInterface> mUltrasoundInterface;

    RobotVisualizator *mRobotVisualizator;

    RobotManualMoveLayout* mMoveLayout;

    void connectToCamera();
    void disconnectFromCamera();
    void stopStreaming();

    void connectToUltrasound();

    void setupUI();

    QString mGraphicsFolderName;
    QLineEdit *robotIPLineEdit, *cameraMinDepthLineEdit,*cameraMaxDepthLineEdit, *usIPLineEdit;
    QPushButton *robotConnectButton, *robotDisconnectButton, *robotShutdownButton;
    QPushButton *cameraConnectButton, *cameraDisconnectButton;
    QPushButton *usConnectButton, *usDisconnectButton;
    QPushButton *calibrateButton, *registerTargetButton, *moveToolButton;
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
    void moveToolToTarget();

    std::vector<Renderer::pointer> mView3DRenderers;
    std::vector<Renderer::pointer> mView2DRenderers;
    std::vector<Renderer::pointer> mViewUSRenderers;
    void clearRenderVectors();



    void updateRenderers(std::vector<Renderer::pointer> view3DRenderers,
                         std::vector<Renderer::pointer> view2DRenderers = std::vector<Renderer::pointer>(),
                         std::vector<Renderer::pointer> viewUSRenderers = std::vector<Renderer::pointer>());
};

}

#endif
