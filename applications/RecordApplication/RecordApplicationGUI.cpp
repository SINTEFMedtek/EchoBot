#include "RecordApplicationGUI.hpp"

#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QTabWidget>
#include <QFileDialog>

#include <Eigen/Dense>

#include <FAST/Visualization/VertexRenderer/VertexRenderer.hpp>
#include "FAST/Visualization/ImageRenderer/ImageRenderer.hpp"
#include "FAST/Streamers/RealSenseStreamer.hpp"

namespace echobot {

RecordApplicationGUI::RecordApplicationGUI()
{
    mCameraInterface = CameraInterface::New();
    mUltrasoundInterface = UltrasoundInterface::New();

    setupUI();
    setupConnections();
}

void RecordApplicationGUI::setupConnections()
{
    QObject::connect(mConnectionWidget, &ConnectionWidget::cameraConnected, std::bind(&RecordApplicationGUI::connectToCamera, this));
    QObject::connect(mConnectionWidget, &ConnectionWidget::cameraDisconnected, std::bind(&RecordApplicationGUI::disconnectFromCamera, this));

    QObject::connect(mConnectionWidget, &ConnectionWidget::usConnected, std::bind(&RecordApplicationGUI::connectToUltrasound, this));
    QObject::connect(mConnectionWidget, &ConnectionWidget::usDisconnected, std::bind(&RecordApplicationGUI::disconnectFromUltrasound, this));

    connect(mRecordWidget, &RecordWidget::playbackStarted, this, &RecordApplicationGUI::playbackButtonSlot);
    connect(mRecordWidget, &RecordWidget::playbackStopped, this, &RecordApplicationGUI::stopPlaybackButtonSlot);
}

// Camera

void RecordApplicationGUI::connectToCamera() {
    stopComputationThread();
    removeAllRenderers();
    setupCameraVisualization();
    if(mUltrasoundStreaming){
        mUltrasoundInterface->connect();
        setupUltrasoundVisualization();
    }
    mCameraStreaming = true;

    reinitializeViews();
    startComputationThread();
}

void RecordApplicationGUI::disconnectFromCamera() {
    stopComputationThread();
    removeAllRenderers();
    if(mUltrasoundStreaming){
        mUltrasoundInterface->connect();
        setupUltrasoundVisualization();
    }
    mCameraStreaming = false;
    startComputationThread();
}


void RecordApplicationGUI::playbackButtonSlot(std::unordered_map<uint, Streamer::pointer> streamers)
{
    mCameraInterface->getProcessObject()->setInputConnection(0, streamers[0]->getOutputPort());
    mCameraInterface->getProcessObject()->setInputConnection(1, streamers[1]->getOutputPort());
    mUltrasoundInterface->getProcessObject()->setInputConnection(0, streamers[0]->getOutputPort());

    mCameraPlaybackStreamers = streamers;

    stopComputationThread();
    setupCameraVisualization(true);

    startComputationThread();
}

void RecordApplicationGUI::stopPlaybackButtonSlot()
{
    mCameraPlaybackStreamers[0]->stopPipeline();
    mCameraPlaybackStreamers[1]->stopPipeline();
    this->stopStreaming();
}

void RecordApplicationGUI::setupCameraVisualization(bool cameraPlayback) {

//    if(cameraPlayback){
//        for(auto it: mCameraPlaybackStreamers){
//            std::cout << it.first << std::endl;
//            mCameraInterface->getProcessObject()->setInputConnection(it.first, it.second->getOutputPort(0));
//        }
//    }
//    else{
//        mCameraInterface->getProcessObject()->setInputConnection(0, mCameraInterface->getStreamObject()->getOutputPort(0));
//        mCameraInterface->getProcessObject()->setInputConnection(1, mCameraInterface->getStreamObject()->getOutputPort(2));
//    }

    getView(0)->addRenderer(mCameraInterface->getImageRenderer());
    getView(1)->addRenderer(mCameraInterface->getDepthImageRenderer());
}

void RecordApplicationGUI::reinitializeViews()
{
    for(auto view: getViews()){
        view->reinitialize();
    }
}

void RecordApplicationGUI::stopStreaming()
{
    stopComputationThread();
    removeAllRenderers();
    startComputationThread();
}

// Ultrasound
void RecordApplicationGUI::connectToUltrasound() {
    stopComputationThread();
    removeAllRenderers();

    setupUltrasoundVisualization();

    if(mCameraStreaming || mCameraPlayback){
        mCameraInterface->connect();
        setupCameraVisualization();
    }

    mUltrasoundStreaming = true;
    reinitializeViews();
    startComputationThread();
}

void RecordApplicationGUI::disconnectFromUltrasound() {
    stopComputationThread();
    removeAllRenderers();
    if(mCameraStreaming || mCameraPlayback){
        mCameraInterface->connect();
        setupCameraVisualization();
    }
    mUltrasoundStreaming = false;
    startComputationThread();
}

void RecordApplicationGUI::setupUltrasoundVisualization()
{
    getView(2)->addRenderer(mUltrasoundInterface->getRendererObject());
}

// UI Setup

void RecordApplicationGUI::setupUI()
{
    View* view2D = createView();
    View* view3D = createView();
    View* viewUS = createView();

    setTitle("Record application");
    setWidth(1920);
    setHeight(1080);
    enableMaximized();

    viewUS->set2DMode();
    viewUS->setBackgroundColor(Color::White());
    viewUS->setFixedWidth(860);

    view3D->set2DMode();
    view3D->setBackgroundColor(Color::White());
    view3D->setFixedWidth(540);

    view2D->set2DMode();
    view2D->setBackgroundColor(Color::White());
    view2D->setFixedWidth(540);

    QVBoxLayout* menuLayout = new QVBoxLayout;
    menuLayout->setAlignment(Qt::AlignTop);
    int menuWidth = 480;

    // Title label
    QLabel* title = new QLabel;
    title->setText("<div style=\"text-align: center; font-weight: bold; font-size: 24px;\">Record application</div>");
    menuLayout->addWidget(title, 0, Qt::AlignTop);


    mConnectionWidget = new ConnectionWidget(menuWidth);
    mConnectionWidget->addInterface(mUltrasoundInterface);
    mConnectionWidget->addInterface(mCameraInterface);
    menuLayout->addWidget(mConnectionWidget, 0, Qt::AlignTop);

    mRecordWidget = new RecordWidget(mCameraInterface, mUltrasoundInterface, menuWidth, 400);
    menuLayout->addWidget(mRecordWidget, 0, Qt::AlignTop);
    menuLayout->addStretch();

    // Quit button
    QPushButton* quitButton = new QPushButton;
    quitButton->setText("Quit (q)");
    quitButton->setStyleSheet("QPushButton { background-color: red; color: white; }");
    QObject::connect(quitButton, &QPushButton::clicked, std::bind(&Window::stop, this));
    menuLayout->addWidget(quitButton,0,Qt::AlignBottom);

    QHBoxLayout* layout = new QHBoxLayout;
    layout->addLayout(menuLayout);
    layout->addWidget(viewUS);

    QWidget* imageWidget = new QWidget;
    QVBoxLayout* vLayout = new QVBoxLayout;
    imageWidget->setLayout(vLayout);
    vLayout->addWidget(view2D);
    vLayout->addWidget(view3D);

    layout->addWidget(imageWidget);
    mWidget->setLayout(layout);
}

}