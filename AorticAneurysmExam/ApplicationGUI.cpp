#include "ApplicationGUI.hpp"

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
#include <FAST/Visualization/LineRenderer/LineRenderer.hpp>
#include <FAST/Streamers/MeshFileStreamer.hpp>
#include <FAST/Streamers/ImageFileStreamer.hpp>
#include <FAST/Exporters/VTKMeshFileExporter.hpp>
#include <FAST/Importers/VTKMeshFileImporter.hpp>
#include "FAST/Visualization/TriangleRenderer/TriangleRenderer.hpp"
#include "FAST/Streamers/KinectStreamer.hpp"
#include "FAST/Visualization/ImageRenderer/ImageRenderer.hpp"

namespace fast {

class MouseListener : public QObject {

public:
    MouseListener(CameraInterface::pointer tracking, View* view);
protected:
    bool eventFilter(QObject *obj, QEvent *event);
private:
    CameraInterface::pointer mCameraInterface;
    Vector2i mPreviousMousePosition;
    View* mView;
};

ApplicationGUI::ApplicationGUI() :
    mGraphicsFolderName("AorticAneurysmExam/widgets/icons/")
{
    mRobotInterface = RobotInterfacePtr(new RobotInterface);

    mRobotVisualizator = new RobotVisualizator();
    mRobotVisualizator->setInterface(mRobotInterface);

    mCameraStreamer = KinectStreamer::New();

    setupUI();
    setupConnections();

    getView(1)->addRenderer(mRobotVisualizator->getRenderer());
    getView(1)->addRenderer(mRobotVisualizator->getRenderer());

    QTimer* timer = new QTimer(this);
    timer->start(1);
    timer->setSingleShot(true);
    QObject::connect(timer, &QTimer::timeout, getView(1), [=]{
        stopComputationThread();
        getView(1)->setLookAt(Vector3f(0, 0, -1000), Vector3f(0, 0, 1000), Vector3f(0, -1, 0), 500, 4000);
        getView(1)->removeAllRenderers();
        startComputationThread();
    });

}

void ApplicationGUI::setupConnections()
{
    connect(robotConnectButton,&QPushButton::clicked,this,&ApplicationGUI::robotConnectButtonSlot);
    connect(robotDisconnectButton,&QPushButton::clicked,this,&ApplicationGUI::robotDisconnectButtonSlot);
    connect(robotShutdownButton,&QPushButton::clicked,this,&ApplicationGUI::robotShutdownButtonSlot);

    QObject::connect(cameraConnectButton, &QPushButton::clicked, std::bind(&ApplicationGUI::connectToCamera, this));
    QObject::connect(cameraDisconnectButton, &QPushButton::clicked, std::bind(&ApplicationGUI::disconnectFromCamera, this));

    QObject::connect(cameraMinDepthLineEdit, &QLineEdit::textChanged, std::bind(&ApplicationGUI::updateCameraROI, this));
    QObject::connect(cameraMaxDepthLineEdit, &QLineEdit::textChanged, std::bind(&ApplicationGUI::updateCameraROI, this));

    QObject::connect(usConnectButton, &QPushButton::clicked, std::bind(&ApplicationGUI::connectToUltrasound, this));

    QObject::connect(calibrateButton, &QPushButton::clicked, std::bind(&ApplicationGUI::calibrateSystem, this));
    QObject::connect(registerTargetButton, &QPushButton::clicked, std::bind(&ApplicationGUI::registerTarget, this));
    QObject::connect(moveToolButton, &QPushButton::clicked, std::bind(&ApplicationGUI::moveToolToTarget, this));
}


// Robot

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

    stopComputationThread();
    clearRenderVectors();
    setupRobotManipulatorVisualization();

    if(mCameraStreaming)
        setupCameraVisualization();

    if(mUltrasoundStreamer)
        setupUltrasoundVisualization();

    // TODO: Add camera and ultrasound if available..
    updateRenderers(mView3DRenderers, mView2DRenderers, mViewUSRenderers);
    startComputationThread();
}

void ApplicationGUI::robotDisconnectButtonSlot()
{
    stopComputationThread();

    mRobotInterface->robot.disconnectFromRobot();

    if(!mRobotInterface->robot.isConnected() && robotConnectButton->isChecked())
        robotConnectButton->toggle();

    getView(1)->removeAllRenderers();
    // TODO: Add camera and ultrasound if available..

    startComputationThread();
}

void ApplicationGUI::robotShutdownButtonSlot()
{
    mRobotInterface->robot.shutdown();
}

void ApplicationGUI::setupRobotManipulatorVisualization()
{
    mView3DRenderers.push_back(mRobotVisualizator->getRenderer());
    mView3DRenderers.push_back(mRobotVisualizator->getTool().getRenderer());
}

void ApplicationGUI::clearRenderVectors()
{
    mView3DRenderers.clear();
    mView2DRenderers.clear();
    mViewUSRenderers.clear();
}

// Camera

void ApplicationGUI::connectToCamera() {
    cameraConnectButton->setChecked(0);

    stopComputationThread();

    mCameraStreamer = KinectStreamer::New();
    mCameraStreamer->getReporter().setReportMethod(Reporter::COUT);
    mCameraStreamer->setPointCloudFiltering(true);
    mCameraStreamer->setMaxRange(2); // All points above x meters are excluded

    // Tracking
    mCameraInterface = CameraInterface::New();
    mCameraInterface->setInputConnection(0, mCameraStreamer->getOutputPort(0));
    mCameraInterface->setInputConnection(1, mCameraStreamer->getOutputPort(2));

    clearRenderVectors();
    setupCameraVisualization();

    if(mRobotInterface->robot.isConnected())
        setupRobotManipulatorVisualization();

    updateRenderers(mView3DRenderers, mView2DRenderers, mViewUSRenderers);

    mCameraStreaming = true;
    startComputationThread();
}

void ApplicationGUI::setupCameraVisualization() {
    // Renderer RGB image
    ImageRenderer::pointer imageRenderer = ImageRenderer::New();
    imageRenderer->addInputConnection(mCameraInterface->getOutputPort(0));

    // Renderer annotations
    SegmentationRenderer::pointer annotationRenderer = SegmentationRenderer::New();
    annotationRenderer->addInputConnection(mCameraInterface->getOutputPort(1));
    annotationRenderer->setFillArea(false);

    // Renderer point cloud
    VertexRenderer::pointer cloudRenderer = VertexRenderer::New();
    cloudRenderer->addInputConnection(mCameraInterface->getOutputPort(2));
    cloudRenderer->setDefaultSize(1.5);

    mView3DRenderers.push_back(cloudRenderer);
    mView2DRenderers.push_back(imageRenderer);
    mView2DRenderers.push_back(annotationRenderer);

    getView(0)->installEventFilter(new MouseListener(mCameraInterface, getView(0)));
}

void ApplicationGUI::updateRenderers(std::vector<Renderer::pointer> mView3DRenderers,
                                     std::vector<Renderer::pointer> mView2DRenderers,
                                     std::vector<Renderer::pointer> mViewUSRenderers)
{
    getView(0)->removeAllRenderers();
    getView(0)->set2DMode();
    getView(0)->setBackgroundColor(Color::White());
    for(auto it: mView2DRenderers)
        getView(0)->addRenderer(it);
    getView(0)->reinitialize();

    getView(1)->removeAllRenderers();
    getView(1)->set3DMode();
    getView(1)->setBackgroundColor(Color::White());

    for(auto it: mView3DRenderers){
        getView(1)->addRenderer(it);
    }

    getView(1)->reinitialize();

    getView(2)->removeAllRenderers();
    getView(2)->set2DMode();
    getView(2)->setBackgroundColor(Color::White());

    for(auto it: mViewUSRenderers){
        getView(2)->addRenderer(it);
    }

    getView(2)->reinitialize();
}


void ApplicationGUI::disconnectFromCamera() {
    stopComputationThread();
    mCameraStreamer->stop();
    mCameraStreaming = false;
    startComputationThread();
}

void ApplicationGUI::updateCameraROI(){
    mCameraStreamer->setMinRange(cameraMinDepthLineEdit->text().toFloat()/100);
    mCameraStreamer->setMaxRange(cameraMaxDepthLineEdit->text().toFloat()/100);
}

void ApplicationGUI::restartCamera() {

    stopComputationThread();
    setupCameraVisualization();

    if(mRobotInterface->robot.isConnected())
        setupRobotManipulatorVisualization();

    updateRenderers(mView3DRenderers, mView2DRenderers, mViewUSRenderers);
    startComputationThread();
}

void ApplicationGUI::stopStreaming()
{
    stopComputationThread();
    getView(0)->removeAllRenderers();
    getView(1)->removeAllRenderers();
    getView(2)->removeAllRenderers();
    startComputationThread();
}

// Ultrasound
void ApplicationGUI::connectToUltrasound() {
    usConnectButton->setChecked(0);

    stopComputationThread();
    clearRenderVectors();

    mUltrasoundStreamer = IGTLinkStreamer::New();
    mUltrasoundStreamer->setConnectionAddress(usIPLineEdit->text().toStdString());
    mUltrasoundStreamer->setConnectionPort(18944);

    mUltrasoundInterface = UltrasoundInterface::New();
    mUltrasoundInterface->setInputConnection(mUltrasoundStreamer->getOutputPort());
    mUltrasoundInterface->setRobotInterface(mRobotInterface);

    setupUltrasoundVisualization();

    if(mRobotInterface->robot.isConnected())
        setupRobotManipulatorVisualization();

    if(mCameraStreaming)
        setupCameraVisualization();

    updateRenderers(mView3DRenderers, mView2DRenderers, mViewUSRenderers);

    mUltrasoundStreaming = true;
    startComputationThread();
}

void ApplicationGUI::setupUltrasoundVisualization()
{
    ImageRenderer::pointer usRenderer = ImageRenderer::New();
    usRenderer->addInputConnection(mUltrasoundInterface->getOutputPort());

    mViewUSRenderers.push_back(usRenderer);
    mView3DRenderers.push_back(usRenderer);
}

// Calibration
void ApplicationGUI::calibrateSystem()
{
    Eigen::Affine3d rMb = Eigen::Affine3d::Identity();
    Eigen::Vector3d translation(-500,350,1000);

    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(0.55*M_PI, Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())*
        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());

    rMb.translate(translation);
    rMb.linear() = rMb.linear()*m;

    mRobotInterface->robot.set_rMb(rMb);

    Eigen::Affine3d eeMt = Eigen::Affine3d::Identity();
    //Eigen::Vector3d translation(0,0,500);

    eeMt.translate(Eigen::Vector3d(-120,0,150));

    mRobotInterface->robot.set_eeMt(eeMt);
}


// Registration
void ApplicationGUI::registerTarget()
{
    if(mTargetRegistered)
    {
        mCameraInterface->removeTargetCloud();
        registerTargetButton->setText("Register target");
        restartCamera();
    }
    else
    {
        this->extractPointCloud();
        registerTargetButton->setText("Remove registration");
    }
    mTargetRegistered = !mTargetRegistered;
}

// Motion

Eigen::MatrixXf vertexVectorToMatrix(std::vector<MeshVertex> vertexVector)
{
    MatrixXf points = Eigen::MatrixXf::Zero(3, vertexVector.size());
    for(int i = 0; i< vertexVector.size(); i++){
        points.col(i) = vertexVector[i].getPosition();
    }
    return points;
}

Vector3f getCentroid(const MatrixXf m) {
    return m.rowwise().sum() / m.cols();
}


void ApplicationGUI::moveToolToTarget()
{
    if(mMovingToTarget)
    {
        moveToolButton->setText("Move to target");
        mRobotInterface->robot.stopMove(corah::MotionType::stopj, 50);
    }else{
        Mesh::pointer targetCloud = mCameraInterface->getTargetCloud();
        MeshAccess::pointer targetCloudAccess = targetCloud->getMeshAccess(ACCESS_READ);
        std::vector<MeshVertex> targetCloudVertices = targetCloudAccess->getVertices();
        Eigen::MatrixXf targetCloudVerticesMat = vertexVectorToMatrix(targetCloudVertices);
        Vector3f pointCloudCentroid = getCentroid(targetCloudVerticesMat);

        Eigen::Affine3d rMtarget = Eigen::Affine3d::Identity();
        rMtarget.translation() = pointCloudCentroid.cast<double>();

        Eigen::Affine3d new_bMee = mRobotInterface->robot.get_rMb().inverse()*rMtarget*mRobotInterface->robot.get_eeMt().inverse();
        new_bMee.linear() = mRobotInterface->robot.getCurrentState().bMee.linear();

        mRobotInterface->robot.move(corah::MotionType::movep, new_bMee, 50, 25);
        moveToolButton->setText("Abort move");
    }
    mMovingToTarget = !mMovingToTarget;
}


// Recording

void ApplicationGUI::toggleRecord() {
    mRecording = !mRecording;
    if(mRecording) {
        mRecordButton->setText("Stop recording");
        mRecordButton->setStyleSheet("QPushButton { background-color: red; color: white; }");
        mStorageDir->setDisabled(true);
        mRecordTimer->start();
        extractPointCloud();
    } else {
        mRecordButton->setText("Record");
        mRecordButton->setStyleSheet("QPushButton { background-color: green; color: white; }");
        mStorageDir->setDisabled(false);
        mCameraInterface->stopRecording();
        refreshRecordingsList();
    }
}

void ApplicationGUI::refreshRecordingsList() {
    // Get all folders in the folder mStorageDir
    QDirIterator it(mStorageDir->text());
    mRecordingsList->clear();
    while(it.hasNext()) {
        it.next();
        QString next = it.fileName();
        if(next.size() > 4)
            mRecordingsList->addItem(next);
    }
}

void ApplicationGUI::playRecording() {
    mPlaying = !mPlaying;
    if(!mPlaying) {
        mPlayButton->setText("Play");
        mPlayButton->setStyleSheet("QPushButton { background-color: green; color: white; }");
        stopStreaming();
    } else {
        auto selectedItems = mRecordingsList->selectedItems();
        if(selectedItems.size() == 0) {
            // Show error message
            QMessageBox *message = new QMessageBox;
            message->setWindowTitle("Error");
            message->setText("You did not select a recording.");
            message->show();
            return;
        }

        mCameraStreamer->stop();

        std::string selectedRecording = (
                mStorageDir->text() +
                QDir::separator() +
                selectedItems[0]->text() +
                QDir::separator()
        ).toUtf8().constData();

        std::string selectedRecordingPointClouds = selectedRecording + "/PointClouds/";
        std::string selectedRecordingImages = selectedRecording + "/CameraImages/";

        // Set up streaming from disk
        auto meshStreamer = MeshFileStreamer::New();
        meshStreamer->setFilenameFormat(selectedRecordingPointClouds + "#.vtk");
        meshStreamer->enableLooping();
        meshStreamer->update(0);

        auto imageStreamer = ImageFileStreamer::New();
        imageStreamer->setFilenameFormat(selectedRecordingImages + "Cam-2D_#.mhd");
        imageStreamer->enableLooping();
        imageStreamer->setSleepTime(100);
        imageStreamer->update(0);

        mCameraInterface = CameraInterface::New();
        mCameraInterface->setInputConnection(0, imageStreamer->getOutputPort());
        mCameraInterface->setInputConnection(1, meshStreamer->getOutputPort());

        stopComputationThread();
        clearRenderVectors();

        setupCameraVisualization();
        if(mRobotInterface->robot.isConnected())
            setupRobotManipulatorVisualization();

        updateRenderers(mView3DRenderers, mView2DRenderers, mViewUSRenderers);

        startComputationThread();

        mPlayButton->setText("Stop");
        mPlayButton->setStyleSheet("QPushButton { background-color: red; color: white; }");
    }
}


void ApplicationGUI::extractPointCloud() {
    stopComputationThread();

    mCameraInterface->calculateTargetCloud(mCameraStreamer);
    mCameraInterface->setInputConnection(0, mCameraStreamer->getOutputPort(0));
    mCameraInterface->setInputConnection(1, mCameraStreamer->getOutputPort(2));

    // If recording is enabled: Store the target cloud, then activate recording on tracking object
    if(mRecording) {
        // Create recording path
        std::string path = mStorageDir->text().toUtf8().constData();
        if(mRecordingNameLineEdit->text() != "") {
            mRecordingName =  currentDateTime() + " " + mRecordingNameLineEdit->text().toUtf8().constData();
        } else {
            mRecordingName = currentDateTime();
        }
        std::string recordingPath = (QString(path.c_str()) + QDir::separator() + QString(mRecordingName.c_str()) + QDir::separator()).toUtf8().constData();
        createDirectories(recordingPath);

        std::cout << "Getting ready to start recording..." << std::endl;
        // Start saving point clouds
        mCameraInterface->startRecording(recordingPath);
    }

    // Renderer RGB image
    ImageRenderer::pointer imageRenderer = ImageRenderer::New();
    imageRenderer->addInputConnection(mCameraInterface->getOutputPort(0));

    // Renderer point cloud
    VertexRenderer::pointer cloudRenderer = VertexRenderer::New();
    uint port = cloudRenderer->addInputConnection(mCameraInterface->getOutputPort(2));
    cloudRenderer->setDefaultSize(1.5);
    cloudRenderer->setColor(port, Color::Green());
    cloudRenderer->addInputConnection(mCameraStreamer->getOutputPort(2));

    mView3DRenderers = {cloudRenderer};
    mView2DRenderers = {imageRenderer};

    if(mRobotInterface->robot.isConnected())
        setupRobotManipulatorVisualization();

    updateRenderers(mView3DRenderers, mView2DRenderers, mViewUSRenderers);

    startComputationThread();
}


// UI Setup

void ApplicationGUI::setupUI()
{
    View* view2D = createView();
    View* view3D = createView();
    View* viewUS = createView();

    const int menuWidth = 300;

    setTitle("Aortic Aneurysm Exam");
    setWidth(1920);
    setHeight(1080);
    enableMaximized();

    view3D->set3DMode();
    view3D->setBackgroundColor(Color::White());
    view3D->setFixedWidth(720);

    view2D->set2DMode();
    view2D->setBackgroundColor(Color::White());
    view2D->setFixedWidth(550);

    viewUS->set2DMode();
    viewUS->setBackgroundColor(Color::White());
    viewUS->setFixedWidth(550);

    QVBoxLayout* menuLayout = new QVBoxLayout;
    menuLayout->setAlignment(Qt::AlignTop);

    // Title label
    QLabel* title = new QLabel;
    title->setText("<div style=\"text-align: center; font-weight: bold; font-size: 24px;\">Aortic Aneurysm Exam</div>");
    menuLayout->addWidget(title);

    QTabWidget *connectionsTabWidget = new QTabWidget;
    QWidget *robotConnectionWidget = getRobotConnectionWidget();
    QWidget *cameraConnectionWidget = getCameraConnectionWidget();
    QWidget *usConnectionWidget = getUltrasoundConnectionWidget();

    connectionsTabWidget->addTab(robotConnectionWidget, "Robot");
    connectionsTabWidget->addTab(cameraConnectionWidget, "Camera");
    connectionsTabWidget->addTab(usConnectionWidget, "Ultrasound");
    menuLayout->addWidget(connectionsTabWidget);

    mMoveLayout = new RobotManualMoveLayout(mRobotInterface);

    QWidget *workflowWidget = getWorkflowWidget();
    tabWidget = new QTabWidget;
    tabWidget->addTab(workflowWidget, "Workflow");
    tabWidget->addTab(mMoveLayout->tabWindow, "Robot manual motion");
    menuLayout->addWidget(tabWidget);

    QWidget *recordingWidget = getRecordingWidget();
    menuLayout->addWidget(recordingWidget);


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

    QWidget* imageWidget = new QWidget;
    QVBoxLayout* vLayout = new QVBoxLayout;
    imageWidget->setLayout(vLayout);
    vLayout->addWidget(view2D);
    vLayout->addWidget(viewUS);

    layout->addWidget(imageWidget);
    mWidget->setLayout(layout);
}

QWidget* ApplicationGUI::getCameraConnectionWidget()
{
    QWidget *group = new QWidget;

    QGridLayout *mainLayout = new QGridLayout();
    group->setLayout(mainLayout);

    cameraConnectButton = new QPushButton();
    cameraMinDepthLineEdit = new QLineEdit();
    cameraMaxDepthLineEdit = new QLineEdit();
    cameraMinDepthLineEdit->setText(QString("0"));
    cameraMaxDepthLineEdit->setText(QString("200"));

    mainLayout->addWidget(new QLabel("Min Depth [cm]: "), 0, 0, 1, 1);
    mainLayout->addWidget(cameraMinDepthLineEdit,0,1,1,1);
    mainLayout->addWidget(cameraConnectButton,0,2,1,1);

    mainLayout->addWidget(new QLabel("Max Depth [cm]: "), 1, 0, 1, 1);
    mainLayout->addWidget(cameraMaxDepthLineEdit,1,1,1,1);

    QIcon icon;
    icon.addFile(mGraphicsFolderName+"network-idle.ico", QSize(), QIcon::Normal, QIcon::Off);
    icon.addFile(mGraphicsFolderName+"network-transmit-receive.ico", QSize(), QIcon::Normal, QIcon::On);
    cameraConnectButton->setIcon(icon);
    cameraConnectButton->setToolTip("Connect to robot");
    cameraConnectButton->setText("Connect");
    cameraConnectButton->setCheckable(true);
    cameraConnectButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    cameraDisconnectButton = new QPushButton(QIcon(mGraphicsFolderName+"network-offline.ico"),"Disconnect");
    mainLayout->addWidget(cameraDisconnectButton,1,2,1,1);

    return group;
}

QWidget* ApplicationGUI::getRobotConnectionWidget()
{
    QWidget *group = new QWidget;
    QGridLayout *mainLayout = new QGridLayout();
    group->setLayout(mainLayout);

    int row = 0;
    robotIPLineEdit = new QLineEdit();
    robotConnectButton = new QPushButton();
    mainLayout->addWidget(new QLabel("IP Address: "), row, 0, 1, 1);
    mainLayout->addWidget(robotIPLineEdit, row, 1,1,1);
    mainLayout->addWidget(robotConnectButton,row,2,1,1);

    robotIPLineEdit->setText("10.218.140.123"); // 10.218.140.114
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

    return group;
}

QWidget* ApplicationGUI::getUltrasoundConnectionWidget()
{
    QWidget *group = new QWidget;

    QGridLayout *mainLayout = new QGridLayout();
    group->setLayout(mainLayout);

    int row = 0;
    usIPLineEdit = new QLineEdit();
    usConnectButton = new QPushButton();
    mainLayout->addWidget(new QLabel("IP Address: "), row, 0, 1, 1);
    mainLayout->addWidget(usIPLineEdit, row, 1,1,1);
    mainLayout->addWidget(usConnectButton,row,2,1,1);

    usIPLineEdit->setText("192.168.140.116"); // 10.218.140.114
    usIPLineEdit->setAlignment(Qt::AlignCenter);

    QIcon icon;
    icon.addFile(mGraphicsFolderName+"network-idle.ico", QSize(), QIcon::Normal, QIcon::Off);
    icon.addFile(mGraphicsFolderName+"network-transmit-receive.ico", QSize(), QIcon::Normal, QIcon::On);
    usConnectButton->setIcon(icon);
    usConnectButton->setToolTip("Connect to US Scanner");
    usConnectButton->setText("Connect");
    usConnectButton->setCheckable(true);
    usConnectButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    row++;
    usDisconnectButton = new QPushButton(QIcon(mGraphicsFolderName+"network-offline.ico"),"Disconnect");
    mainLayout->addWidget(usDisconnectButton,row,2,1,1);

    return group;
}


QWidget* ApplicationGUI::getRecordingWidget()
{
    QGroupBox* group = new QGroupBox("Record exam");
    group->setFlat(true);

    QGridLayout *mainLayout = new QGridLayout();
    group->setLayout(mainLayout);

    mRecordTimer = new QElapsedTimer;

    QLabel* storageDirLabel = new QLabel;
    storageDirLabel->setText("Storage directory:");
    mainLayout->addWidget(storageDirLabel, 0, 0, 1, 1);

    mStorageDir = new QLineEdit;
    mStorageDir->setText(QDir::homePath() + QDir::separator() + QString("FAST_Kinect_Recordings"));
    mainLayout->addWidget(mStorageDir, 0, 1, 1, 1);

    QLabel* recordingNameLabel = new QLabel;
    recordingNameLabel->setText("Subject name:");
    mainLayout->addWidget(recordingNameLabel, 1, 0, 1, 1);

    mRecordingNameLineEdit = new QLineEdit;
    mainLayout->addWidget(mRecordingNameLineEdit, 1, 1, 1, 1);

    mRecordButton = new QPushButton;
    mRecordButton->setText("Record");
    mRecordButton->setStyleSheet("QPushButton { background-color: green; color: white; }");
    QObject::connect(mRecordButton, &QPushButton::clicked, std::bind(&ApplicationGUI::toggleRecord, this));
    mainLayout->addWidget(mRecordButton, 2, 0, 1, 1);

    mPlayButton = new QPushButton;
    mPlayButton->setText("Play");
    mPlayButton->setStyleSheet("QPushButton { background-color: green; color: white; }");
    mainLayout->addWidget(mPlayButton, 2, 1, 1, 1);
    QObject::connect(mPlayButton, &QPushButton::clicked, std::bind(&ApplicationGUI::playRecording, this));

    mRecordingInformation = new QLabel;
    mRecordingInformation->setStyleSheet("QLabel { font-size: 14px; }");
    mainLayout->addWidget(mRecordingInformation);

    mRecordingsList = new QListWidget;
    mainLayout->addWidget(mRecordingsList, 3, 0, 1, 2);
    mRecordingsList->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    mRecordingsList->setFixedHeight(100);
    mRecordingsList->setSortingEnabled(true);
    refreshRecordingsList();

    return group;
}

QWidget* ApplicationGUI::getWorkflowWidget()
{
    QWidget *group = new QWidget;

    QGridLayout *mainLayout = new QGridLayout();
    group->setLayout(mainLayout);

    int row = 0;
    calibrateButton = new QPushButton();
    mainLayout->addWidget(calibrateButton,row,2,1,1);
    calibrateButton->setText("Calibrate");
    calibrateButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    row++;
    registerTargetButton = new QPushButton();
    mainLayout->addWidget(registerTargetButton,row,2,1,1);
    registerTargetButton->setText("Register target");
    registerTargetButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    row++;
    moveToolButton = new QPushButton();
    mainLayout->addWidget(moveToolButton,row,2,1,1);
    moveToolButton->setText("Move to target");
    moveToolButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    return group;
}

MouseListener::MouseListener(CameraInterface::pointer cameraInterface, View* view) : QObject(view) {
    mCameraInterface = cameraInterface;
    mPreviousMousePosition = Vector2i(-1, -1);
    mView = view;
}

bool MouseListener::eventFilter(QObject *obj, QEvent *event) {
    if(event->type() == QEvent::MouseButtonRelease) {
        mPreviousMousePosition = Vector2i(-1, -1);
    }
    if(event->type() == QEvent::MouseMove) {
        // Releay mouse movement to tracking
        QMouseEvent *mouseEvent = static_cast<QMouseEvent *>(event);
        // TODO need to go from view coordinates to physical to image coordinates
        const Matrix4f perspectiveMatrix = mView->getPerspectiveMatrix();
        const Matrix4f viewMatrix = mView->getViewMatrix();
        Vector3f current(2.0f*(float)mouseEvent->x()/mView->width() - 1.0f, -(2.0f*(float)mouseEvent->y()/mView->height() -1.0f), 0);
        current = (viewMatrix.inverse()*perspectiveMatrix.inverse()*current.homogeneous()).head(3);
        if(mPreviousMousePosition.x() == -1 && mPreviousMousePosition.y() == -1) {
            mPreviousMousePosition = current.cast<int>().head(2);
        } else {
            mCameraInterface->addLine(mPreviousMousePosition, current.cast<int>().head(2));
            mPreviousMousePosition = current.cast<int>().head(2);
        }
    }

    // standard event processing
    return QObject::eventFilter(obj, event);
}


}

//    Mesh::pointer mesh = Mesh::New();
//    std::vector<MeshVertex> vertices = {
//            MeshVertex(Vector3f(0, 0, 0)),
//            MeshVertex(Vector3f(25, 0, 0)),
//            MeshVertex(Vector3f(0, 25, 0)),
//            MeshVertex(Vector3f(0, 0, 25)),
//    };
//    std::vector<MeshLine> lines = {
//            MeshLine(0, 1),
//            MeshLine(0, 2),
//            MeshLine(0, 3),
//    };
//    mesh->create(vertices, lines);
//
//    LineRenderer::pointer lineRenderer = LineRenderer::New();
//    lineRenderer->addInputData(mesh);
//    lineRenderer->setDefaultLineWidth(20);
//    lineRenderer->setColor(0, Color::Red());