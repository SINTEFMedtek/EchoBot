#include "ApplicationGUI.hpp"
#include "KinectTracking.hpp"
#include "FAST/Streamers/KinectStreamer.hpp"
#include "FAST/Visualization/ImageRenderer/ImageRenderer.hpp"
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QDir>
#include <QElapsedTimer>
#include <QListWidget>
#include <QDirIterator>
#include <QMessageBox>
#include <QGroupBox>
#include <QProgressDialog>
#include <QDir>
#include <QTabWidget>

#include "FAST/Visualization/MultiViewWindow.hpp"
#include <FAST/Visualization/SegmentationRenderer/SegmentationRenderer.hpp>
#include <FAST/Visualization/VertexRenderer/VertexRenderer.hpp>
#include <FAST/Streamers/MeshFileStreamer.hpp>
#include <FAST/Exporters/VTKMeshFileExporter.hpp>
#include <FAST/Importers/VTKMeshFileImporter.hpp>
#include "FAST/Visualization/TriangleRenderer/TriangleRenderer.hpp"

namespace fast {

ApplicationGUI::ApplicationGUI() :
        mGraphicsFolderName("AorticAneurysmExam/widgets/icons/")
{
    //QDir dir = QDir::current(); dir.cdUp(); dir.cdUp();
    //mGraphicsFolderName = dir.path()+mGraphicsFolderName;

    mRobotInterface = RobotInterfacePtr(new RobotInterface);

    setupUI();
    setupConnections();
}

void ApplicationGUI::setupConnections()
{
    connect(robotConnectButton,&QPushButton::clicked,this,&ApplicationGUI::robotConnectButtonSlot);
    connect(robotDisconnectButton,&QPushButton::clicked,this,&ApplicationGUI::robotDisconnectButtonSlot);
    connect(robotShutdownButton,&QPushButton::clicked,this,&ApplicationGUI::robotShutdownButtonSlot);

    QObject::connect(cameraConnectButton, &QPushButton::clicked, std::bind(&ApplicationGUI::connectToCamera, this));
    QObject::connect(cameraDisconnectButton, &QPushButton::clicked, std::bind(&ApplicationGUI::disconnectFromCamera, this));
}

void ApplicationGUI::robotConnectButtonSlot()
{
    mRobotInterface->robot.configure(corah::Manipulator::UR5,robotIPLineEdit->text(),30003);
    mRobotInterface->robot.start();

    if(mRobotInterface->robot.isConnected() && !robotConnectButton->isChecked())
    {
        robotConnectButton->toggle();
    }
    else if(!mRobotInterface->robot.isConnected() && robotConnectButton->isChecked())
    {
        robotConnectButton->toggle();
    }

    View* view3D = getView(1);
    stopComputationThread();
    setupRobotManipulatorVisualization();
    view3D->reinitialize();
    startComputationThread();
}

void ApplicationGUI::robotDisconnectButtonSlot()
{
    mRobotInterface->robot.disconnectFromRobot();

    if(!mRobotInterface->robot.isConnected() && robotConnectButton->isChecked())
        robotConnectButton->toggle();

}

void ApplicationGUI::robotShutdownButtonSlot()
{
    mRobotInterface->robot.shutdown();
}


void ApplicationGUI::connectToCamera() {
    cameraConnectButton->setChecked(0);

    View* view2D = getView(0);
    View* view3D = getView(1);

    stopComputationThread();
    view2D->removeAllRenderers();
    view3D->removeAllRenderers();

    // Streaming
    mStreamer = KinectStreamer::New();
    mStreamer->getReporter().setReportMethod(Reporter::COUT);
    mStreamer->setPointCloudFiltering(true);
    mStreamer->setMaxRange(2); // All points above x meters are excluded

    // Tracking
    mTracking = KinectTracking::New();
    mTracking->setInputConnection(0, mStreamer->getOutputPort(0));
    mTracking->setInputConnection(1, mStreamer->getOutputPort(2));

    // Renderer RGB image
    ImageRenderer::pointer renderer = ImageRenderer::New();
    renderer->addInputConnection(mTracking->getOutputPort(0));

    // Renderer point cloud
    VertexRenderer::pointer cloudRenderer = VertexRenderer::New();
    cloudRenderer->addInputConnection(mStreamer->getOutputPort(2));
    cloudRenderer->setDefaultSize(1.5);

    view3D->set3DMode();
    view3D->setBackgroundColor(Color::Black());
    view3D->addRenderer(cloudRenderer);
    view3D->reinitialize();

    view2D->set2DMode();
    view2D->setBackgroundColor(Color::Black());
    view2D->addRenderer(renderer);
    view2D->reinitialize();

    setupRobotToolVisualization();
    setupRobotManipulatorVisualization();

    startComputationThread();
}

void ApplicationGUI::disconnectFromCamera() {
    mStreamer->stop();
    mStreamer->stopPipeline();
    mTracking->stopRecording();
    mTracking->stopPipeline();
}

void ApplicationGUI::setupRobotToolVisualization()
{
    RobotTool *tool = new RobotTool("AorticAneurysmExam/visualization/CADModels/5S-Probe.vtk");
    View* view3D = getView(1);
    view3D->addRenderer(tool->getRenderer());
}

void ApplicationGUI::setupRobotManipulatorVisualization() {
    View* view3D = getView(1);

    RobotManipulator *manipulator = new RobotManipulator(mRobotInterface);
    view3D->addRenderer(manipulator->getRenderer(0));
    view3D->addRenderer(manipulator->getRenderer(1));
    view3D->addRenderer(manipulator->getRenderer(2));
    view3D->addRenderer(manipulator->getRenderer(3));
    view3D->addRenderer(manipulator->getRenderer(4));
    view3D->addRenderer(manipulator->getRenderer(5));
    view3D->addRenderer(manipulator->getRenderer(6));
}

void ApplicationGUI::setupUI()
{
    View* view2D = createView();
    View* view3D = createView();

    const int menuWidth = 300;

    setTitle("Aortic Aneurysm Exam");
    setWidth(1920);
    setHeight(1080);
    enableMaximized();

    view3D->set3DMode();
    view3D->setBackgroundColor(Color::Black());
    view3D->setFixedWidth(750);

    view2D->set2DMode();
    view2D->setBackgroundColor(Color::Black());
    view2D->setFixedWidth(550);

    QVBoxLayout* menuLayout = new QVBoxLayout;
    menuLayout->setAlignment(Qt::AlignTop);

    // Title label
    QLabel* title = new QLabel;
    title->setText("<div style=\"text-align: center; font-weight: bold; font-size: 24px;\">Aortic Aneurysm Exam</div>");
    menuLayout->addWidget(title);

    //QPushButton* cameraConnectionButton = new QPushButton;
    //cameraConnectionButton->setText("Connect to camera");
    //cameraConnectionButton->setStyleSheet("QPushButton { background-color: green; color: white; }");
    //cameraConnectionButton->setFixedWidth(menuWidth);
    //menuLayout->addWidget(cameraConnectionButton);

//    QPushButton* robotConnectionButton = new QPushButton;
//    robotConnectionButton->setText("Connect to robot");
//    robotConnectionButton->setStyleSheet("QPushButton {background-color:green; color:white; font-size=36px}");
//    robotConnectionButton->setFixedWidth(menuWidth);
//    QObject::connect(robotConnectionButton, &QPushButton::clicked, std::bind(&ApplicationGUI::toggleRobotConnection, this));
//    menuLayout->addWidget(robotConnectionButton);
//
//    QPushButton* robotVisualizeButton = new QPushButton;
//    robotVisualizeButton->setText("Visualize links");
//    robotVisualizeButton->setStyleSheet("QPushButton { background-color: blue; color: white; }");
//    robotVisualizeButton->setFixedWidth(menuWidth);
//    //QObject::connect(robotVisualizeButton, &QPushButton::clicked, std::bind(&RobotInterface::startVisualization, this));
//    menuLayout->addWidget(robotVisualizeButton);

    setRobotConnectionLayout(menuLayout);
    setCameraConnectionLayout(menuLayout);

    mMoveLayout = new RobotManualMoveLayout(mRobotInterface);

    QWidget *testWidget = new QWidget;

    tabWidget = new QTabWidget;
    tabWidget->addTab(testWidget, "Workflow");
    tabWidget->addTab(mMoveLayout->tabWindow, "Robot manual motion");
    menuLayout->addWidget(tabWidget);

    // Quit button
    QPushButton* quitButton = new QPushButton;
    quitButton->setText("Quit (q)");
    quitButton->setStyleSheet("QPushButton { background-color: red; color: white; }");
    quitButton->setFixedWidth(menuWidth);
    QObject::connect(quitButton, &QPushButton::clicked, std::bind(&Window::stop, this));
    menuLayout->addWidget(quitButton,0,Qt::AlignBottom);

    QHBoxLayout* layout = new QHBoxLayout;
    layout->addLayout(menuLayout);
    layout->addWidget(view3D);
    layout->addWidget(view2D);
    mWidget->setLayout(layout);
}

void ApplicationGUI::setCameraConnectionLayout(QVBoxLayout *parent)
{
    QGroupBox* group = new QGroupBox("Camera");
    group->setFlat(true);
    parent->addWidget(group,0,Qt::AlignTop);

    QGridLayout *mainLayout = new QGridLayout();
    group->setLayout(mainLayout);

    cameraConnectButton = new QPushButton();
    mainLayout->addWidget(cameraConnectButton,0,0,1,1);

    QIcon icon;
    icon.addFile(mGraphicsFolderName+"network-idle.ico", QSize(), QIcon::Normal, QIcon::Off);
    icon.addFile(mGraphicsFolderName+"network-transmit-receive.ico", QSize(), QIcon::Normal, QIcon::On);
    cameraConnectButton->setIcon(icon);
    cameraConnectButton->setToolTip("Connect to robot");
    cameraConnectButton->setText("Connect");
    cameraConnectButton->setCheckable(true);
    cameraConnectButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    cameraDisconnectButton = new QPushButton(QIcon(mGraphicsFolderName+"network-offline.ico"),"Disconnect");
    mainLayout->addWidget(cameraDisconnectButton,0,1,1,1);
}

void ApplicationGUI::setRobotConnectionLayout(QVBoxLayout *parent)
{
    QGroupBox* group = new QGroupBox("Robot");
    group->setFlat(true);
    parent->addWidget(group,0,Qt::AlignTop);

    QGridLayout *mainLayout = new QGridLayout();
    group->setLayout(mainLayout);

    int row = 0;
    robotIPLineEdit = new QLineEdit();
    robotConnectButton = new QPushButton();
    mainLayout->addWidget(new QLabel("IP Address: "), row, 0, 1, 1);
    mainLayout->addWidget(robotIPLineEdit, row, 1,1,1);
    mainLayout->addWidget(robotConnectButton,row,2,1,1);

    robotIPLineEdit->setText("localhost"); // 10.218.140.114
    robotIPLineEdit->setAlignment(Qt::AlignCenter);

    QIcon icon;
    icon.addFile(mGraphicsFolderName+"network-idle.ico", QSize(), QIcon::Normal, QIcon::Off);
    icon.addFile(mGraphicsFolderName+"network-transmit-receive.ico", QSize(), QIcon::Normal, QIcon::On);
    robotConnectButton->setIcon(icon);
    robotConnectButton->setToolTip("Connect to robot");
    robotConnectButton->setText("Connect");
    robotConnectButton->setCheckable(true);
    robotConnectButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    row++;
    robotShutdownButton = new QPushButton(QIcon(mGraphicsFolderName+"application-exit-4.png"),"Shutdown");
    robotDisconnectButton = new QPushButton(QIcon(mGraphicsFolderName+"network-offline.ico"),"Disconnect");
    mainLayout->addWidget(robotShutdownButton,row,0,1,1);
    mainLayout->addWidget(robotDisconnectButton,row,2,1,1);
}


}