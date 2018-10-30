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
#include "FAST/Visualization/MultiViewWindow.hpp"
#include <FAST/Visualization/SegmentationRenderer/SegmentationRenderer.hpp>
#include <FAST/Visualization/VertexRenderer/VertexRenderer.hpp>
#include <FAST/Streamers/MeshFileStreamer.hpp>
#include <FAST/Exporters/VTKMeshFileExporter.hpp>
#include <FAST/Importers/VTKMeshFileImporter.hpp>
#include <QProgressDialog>



namespace fast {

ApplicationGUI::ApplicationGUI() {
    mRobotInterface = RobotInterfacePtr(new RobotInterface);
    setupUI();
}

void ApplicationGUI::toggleRobotConnection() {
    mRobotInterface->robot.configure(corah::Manipulator::UR5,"10.218.140.114",30003);
    mRobotInterface->robot.start();

    if(mRobotInterface->robot.isConnected())
    {
        std::cout << mRobotInterface->robot.getCurrentState().jointConfiguration << std::endl;
    }
}


void ApplicationGUI::toggleCameraConnection() {
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

    startComputationThread();
}


void ApplicationGUI::setupUI()
{
    View* view2D = createView();
    View* view3D = createView();

    const int menuWidth = 300;

    setTitle("Aortic Aneurysm Exam");
    setWidth(1024 + menuWidth);
    setHeight(848);
    enableMaximized();
    view3D->set3DMode();
    view3D->setBackgroundColor(Color::Black());

    view2D->set2DMode();
    view2D->setBackgroundColor(Color::Black());

    QVBoxLayout* menuLayout = new QVBoxLayout;
    menuLayout->setAlignment(Qt::AlignTop);

    // Title label
    QLabel* title = new QLabel;
    title->setText("<div style=\"text-align: center; font-weight: bold; font-size: 24px;\">Aortic Aneurysm Exam</div>");
    menuLayout->addWidget(title);

    // Quit butto1.0f;//n
    QPushButton* quitButton = new QPushButton;
    quitButton->setText("Quit (q)");
    quitButton->setStyleSheet("QPushButton { background-color: red; color: white; }");
    quitButton->setFixedWidth(menuWidth);
    QObject::connect(quitButton, &QPushButton::clicked, std::bind(&Window::stop, this));
    menuLayout->addWidget(quitButton);

    QPushButton* cameraConnectionButton = new QPushButton;
    cameraConnectionButton->setText("Connect to camera");
    cameraConnectionButton->setStyleSheet("QPushButton { background-color: green; color: white; }");
    cameraConnectionButton->setFixedWidth(menuWidth);
    QObject::connect(cameraConnectionButton, &QPushButton::clicked, std::bind(&ApplicationGUI::toggleCameraConnection, this));
    menuLayout->addWidget(cameraConnectionButton);

    QPushButton* robotConnectionButton = new QPushButton;
    robotConnectionButton->setText("Connect to robot");
    robotConnectionButton->setStyleSheet("QPushButton { background-color: green; color: white; }");
    robotConnectionButton->setFixedWidth(menuWidth);
    QObject::connect(robotConnectionButton, &QPushButton::clicked, std::bind(&ApplicationGUI::toggleRobotConnection, this));
    menuLayout->addWidget(robotConnectionButton);

    QPushButton* robotVisualizeButton = new QPushButton;
    robotVisualizeButton->setText("Connect to robot");
    robotVisualizeButton->setStyleSheet("QPushButton { background-color: blue; color: white; }");
    robotVisualizeButton->setFixedWidth(menuWidth);
    //QObject::connect(robotVisualizeButton, &QPushButton::clicked, std::bind(&RobotInterface::startVisualization, this));
    menuLayout->addWidget(robotVisualizeButton);

    mMoveLayout = new RobotManualMoveLayout(mRobotInterface);
    menuLayout->addLayout(mMoveLayout->getLayout());

    QHBoxLayout* layout = new QHBoxLayout;
    layout->addLayout(menuLayout);
    layout->addWidget(view3D);
    layout->addWidget(view2D);
    mWidget->setLayout(layout);
}

}