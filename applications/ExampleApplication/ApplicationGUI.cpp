#include "ApplicationGUI.hpp"

#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QTabWidget>
#include <QFileDialog>

#include <Eigen/Dense>

#include <FAST/Visualization/SegmentationRenderer/SegmentationRenderer.hpp>
#include <FAST/Visualization/VertexRenderer/VertexRenderer.hpp>
#include <FAST/Visualization/LineRenderer/LineRenderer.hpp>
#include <FAST/Importers/VTKMeshFileImporter.hpp>
#include "FAST/Visualization/TriangleRenderer/TriangleRenderer.hpp"
#include "FAST/Streamers/RealSenseStreamer.hpp"
#include "FAST/Visualization/ImageRenderer/ImageRenderer.hpp"
#include "FAST/Importers/ImageFileImporter.hpp"
#include "FAST/Algorithms/SurfaceExtraction/SurfaceExtraction.hpp"
#include "FAST/Algorithms/ImageResizer/ImageResizer.hpp"
#include "FAST/Algorithms/CoherentPointDrift/Rigid.hpp"
#include "FAST/Algorithms/SegmentationVolumeReconstructor/SegmentationVolumeReconstructor.hpp"

#include "EchoBot/GUI/Widgets/ConnectionWidget.h"
#include "EchoBot/Utilities/PointCloudUtilities.h"
#include "EchoBot/Core/Config.h"
#include "EchoBot/Visualization/VisualizationHelper.h"


namespace echobot {

class MouseListener : public QObject {

public:
    MouseListener(CameraInterface::pointer tracking, View* view);
protected:
    bool eventFilter(QObject *obj, QEvent *event);
private:
    CameraInterface::pointer mCameraInterface;
    Eigen::Vector2i mPreviousMousePosition;
    View* mView;
};

ApplicationGUI::ApplicationGUI() :
    mGraphicsFolderName("AorticAneurysmExam/widgets/icons/")
{
    mRobotInterface = RobotInterface::New();
    mCameraInterface = CameraInterface::New();
    mUltrasoundInterface = UltrasoundInterface::New();
    mRobotVisualizator = RobotVisualizator::New();
    mRobotVisualizator->setInterface(mRobotInterface);

    setupUI();
    setupConnections();
    initializeRenderers();
}

void ApplicationGUI::setupConnections()
{
    QObject::connect(mConnectionWidget, &ConnectionWidget::robotConnected, std::bind(&ApplicationGUI::robotConnectButtonSlot, this));
    QObject::connect(mConnectionWidget, &ConnectionWidget::robotDisconnected, std::bind(&ApplicationGUI::robotDisconnectButtonSlot, this));
    QObject::connect(mConnectionWidget, &ConnectionWidget::robotShutdown, std::bind(&ApplicationGUI::robotShutdownButtonSlot, this));

    QObject::connect(mConnectionWidget, &ConnectionWidget::cameraConnected, std::bind(&ApplicationGUI::connectToCamera, this));
    QObject::connect(mConnectionWidget, &ConnectionWidget::cameraDisconnected, std::bind(&ApplicationGUI::disconnectFromCamera, this));

    QObject::connect(mConnectionWidget, &ConnectionWidget::usConnected, std::bind(&ApplicationGUI::connectToUltrasound, this));
    QObject::connect(mConnectionWidget, &ConnectionWidget::usDisconnected, std::bind(&ApplicationGUI::disconnectFromUltrasound, this));

    QObject::connect(mRecordWidget, &RecordWidget::playbackStopped, std::bind(&ApplicationGUI::stopPlaybackButtonSlot, this));
    QObject::connect(mRecordWidget, &RecordWidget::playbackStarted, std::bind(&ApplicationGUI::playbackButtonSlot, this));

    QObject::connect(calibrateButton, &QPushButton::clicked, std::bind(&ApplicationGUI::calibrateSystem, this));
    QObject::connect(registerDataButton, &QPushButton::clicked, std::bind(&ApplicationGUI::registerCloudToData, this));
//    QObject::connect(registerTargetButton, &QPushButton::clicked, std::bind(&ApplicationGUI::registerTarget, this));
//    QObject::connect(moveToolManualButton, &QPushButton::clicked, std::bind(&ApplicationGUI::moveToolToManualTarget, this));
    QObject::connect(planMoveToolRegisteredButton, &QPushButton::clicked, std::bind(&ApplicationGUI::planMoveToRegisteredTarget, this));
    QObject::connect(moveToolRegisteredButton, &QPushButton::clicked, std::bind(&ApplicationGUI::moveToolToRegisteredTarget, this));
    QObject::connect(enableSegmentationButton, &QPushButton::clicked, std::bind(&ApplicationGUI::enableNeuralNetworkSegmentation, this));
}


// Robot

void ApplicationGUI::robotConnectButtonSlot()
{
    stopComputationThread();
    removeAllRenderers();
    setupRobotManipulatorVisualization();

    if(mCameraStreaming || mCameraPlayback){
        mCameraInterface->connect();
        setupCameraVisualization();
    }

    if(mUltrasoundStreaming){
        mUltrasoundInterface->connect();
        setupUltrasoundVisualization();
    }

    reinitializeViews();
    startComputationThread();
}

void ApplicationGUI::robotDisconnectButtonSlot()
{
    stopComputationThread();
    removeAllRenderers();

    if(mUltrasoundStreaming){
        mUltrasoundInterface->connect();
        setupUltrasoundVisualization();
    }

    if(mCameraStreaming || mCameraPlayback){
        mCameraInterface->connect();
        setupCameraVisualization();
    }

    startComputationThread();
}

void ApplicationGUI::robotShutdownButtonSlot()
{
    mRobotInterface->getRobot()->shutdown();
}

void ApplicationGUI::setupRobotManipulatorVisualization()
{
    mRobotVisualizator->setInterface(mRobotInterface);
    getView(1)->addRenderer(mRobotVisualizator->getRenderer());
    getView(1)->addRenderer(mRobotVisualizator->getTool()->getRenderer());
}

// Camera

void ApplicationGUI::connectToCamera() {
    stopComputationThread();
    removeAllRenderers();

    setupCameraVisualization();

    if(mRobotInterface->getRobot()->isConnected()){
        setupRobotManipulatorVisualization();
    }

    if(mUltrasoundStreaming){
        mUltrasoundInterface->connect();
        setupUltrasoundVisualization();
    }

    mCameraStreaming = true;
    reinitializeViews();
    startComputationThread();
}

void ApplicationGUI::disconnectFromCamera() {
    stopComputationThread();
    removeAllRenderers();

    if(mRobotInterface->getRobot()->isConnected()){
        mRobotInterface->getRobot()->connect();
        setupRobotManipulatorVisualization();
    }

    if(mUltrasoundStreaming){
        mUltrasoundInterface->connect();
        setupUltrasoundVisualization();
    }
    mCameraStreaming = false;

    startComputationThread();
}

void ApplicationGUI::setupCameraVisualization() {
    getView(0)->addRenderer(mCameraInterface->getImageRenderer());
    getView(1)->addRenderer(mCameraInterface->getPointCloudRenderer());
}

void ApplicationGUI::restartCamera() {

    stopComputationThread();

    setupCameraVisualization();

    if(mRobotInterface->getRobot()->isConnected())
        setupRobotManipulatorVisualization();

    if(mUltrasoundStreaming)
        setupUltrasoundVisualization();

    startComputationThread();
}

void ApplicationGUI::stopStreaming()
{
    stopComputationThread();
    removeAllRenderers();
    startComputationThread();
}

void ApplicationGUI::playbackButtonSlot()
{
    stopComputationThread();
    removeAllRenderers();

    setupUltrasoundVisualization();
    mUltrasoundStreaming = true;

    setupCameraVisualization();
    mCameraStreaming = true;

    reinitializeViews();
    startComputationThread();
}

void ApplicationGUI::stopPlaybackButtonSlot()
{
    mUltrasoundStreaming = false;
    this->stopStreaming();
}

// Ultrasound
void ApplicationGUI::connectToUltrasound() {
    stopComputationThread();
    removeAllRenderers();
    setupUltrasoundVisualization();

    if(mRobotInterface->getRobot()->isConnected()){
        setupRobotManipulatorVisualization();
    }


    if(mCameraStreaming || mCameraPlayback){
        mCameraInterface->connect();
        setupCameraVisualization();
    }

    mUltrasoundStreaming = true;
    reinitializeViews();
    startComputationThread();
}

void ApplicationGUI::addCoordinateAxis(Eigen::Affine3f transform) {
    mCoordinateAxis.push_back(transform);
}

void ApplicationGUI::renderCoordinateAxis(float axisLength) {
    stopComputationThread();
    removeAllRenderers();

    for(auto transform: mCoordinateAxis)
        getView(1)->addRenderer(VisualizationHelper::createCoordinateFrameRenderer(transform, axisLength));

    if(mRobotInterface->getRobot()->isConnected()){
        mRobotInterface->disconnect();
        mRobotInterface->connect();
        setupRobotManipulatorVisualization();
    }

    if(mCameraStreaming || mCameraPlayback){
        mCameraInterface->connect();
        setupCameraVisualization();
    }

    if(mUltrasoundStreaming){
        mUltrasoundInterface->connect();
        setupUltrasoundVisualization();
    }


    reinitializeViews();
    startComputationThread();
}


void ApplicationGUI::disconnectFromUltrasound() {
    stopComputationThread();
    removeAllRenderers();

    if(mRobotInterface->isConnected())
        setupRobotManipulatorVisualization();

    if(mCameraStreaming || mCameraPlayback){
        mCameraInterface->connect();
        setupCameraVisualization();
    }

    reinitializeViews();
    startComputationThread();
}

void ApplicationGUI::setupUltrasoundVisualization()
{
    getView(1)->addRenderer(mUltrasoundInterface->getImageRenderer());
    getView(2)->addRenderer(mUltrasoundInterface->getImageRenderer());

    if(mUltrasoundInterface->getProcessObject()->isSegmentationEnabled())
        getView(2)->addRenderer(mUltrasoundInterface->getSegmentationRenderer());
}

//// Calibration
void ApplicationGUI::calibrateSystem()
{
    mCalibrationWidget->calibrateSystem();
}

// Registration
void ApplicationGUI::registerTarget()
{
    if(mTargetRegistered)
    {
        mCameraInterface->getProcessObject()->removeTargetCloud();
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

void ApplicationGUI::registerCloudToData()
{
    auto registration_model_file = Config::getRegistrationDataPath() + "accumulated_pc_phantom.vtk";

    auto importer = fast::VTKMeshFileImporter::New();
    importer->setFilename(registration_model_file);

    auto port = importer->getOutputPort();
    importer->update();
    auto registrationCloud = port->getNextFrame<Mesh>();

    DataChannel::pointer dataPort = mCameraInterface->getProcessObject()->getOutputPort(3);
    mCameraInterface->getProcessObject()->update();
    auto currentCloud = dataPort->getNextFrame<Mesh>();

    // Modify point clouds
    auto regCloudReduced = decimateMesh(registrationCloud, (double) 2500.0 / registrationCloud->getNrOfVertices());
    auto currentCloudReduced = decimateMesh(currentCloud, (double) 2500.0 / currentCloud->getNrOfVertices());

    // Set registration settings
    float uniformWeight = 0.5;
    double tolerance = 1e-3;

    std::cout << regCloudReduced->getSceneGraphNode()->getTransformation()->getTransform().matrix()  << "\n" << std::endl;
    std::cout << currentCloudReduced->getSceneGraphNode()->getTransformation()->getTransform().matrix()  << "\n" << std::endl;

    // Run Coherent Point Drift
    auto cpd = fast::CoherentPointDriftRigid::New();
    cpd->setFixedMesh(regCloudReduced);
    cpd->setMovingMesh(currentCloudReduced);
    cpd->setMaximumIterations(50);
    cpd->setTolerance(tolerance);
    cpd->setUniformWeight(uniformWeight);

    auto cpdPort = cpd->getOutputPort();
    cpd->update();
    Mesh::pointer mesh = cpdPort->getNextFrame<Mesh>();
    //mPreoperativeData = mesh;

    Eigen::Affine3f eigtransform = mesh->getSceneGraphNode()->getTransformation()->getTransform().inverse();
    auto calibrationTool = mCalibrationWidget->getCalibrationTool();
    auto pcMct = calibrationTool->get_registration_pcMdata();

    AffineTransformation::pointer transform = AffineTransformation::New();
    transform->setTransform(eigtransform*pcMct.cast<float>());

    mPreoperativeData->getSceneGraphNode()->setTransformation(transform);
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


void ApplicationGUI::moveToolToManualTarget()
{
    if(mMovingToTarget)
    {
        moveToolManualButton->setText("Move to target");
        mRobotInterface->getRobot()->stopMove(romocc::MotionType::stopj, 50);
    }else{
        auto targetCloud = mCameraInterface->getProcessObject()->getTargetCloud();
        auto targetCloudAccess = targetCloud->getMeshAccess(ACCESS_READ);
        std::vector<MeshVertex> targetCloudVertices = targetCloudAccess->getVertices();
        Eigen::MatrixXf targetCloudVerticesMat = vertexVectorToMatrix(targetCloudVertices);
        Vector3f pointCloudCentroid = getCentroid(targetCloudVerticesMat);

        Eigen::Affine3d rMtarget = Eigen::Affine3d::Identity();
        rMtarget.translation() = pointCloudCentroid.cast<double>();

        romocc::RobotCoordinateSystem::pointer coords = mRobotInterface->getRobot()->getCoordinateSystem();
        Eigen::Affine3d new_bMee = coords->get_rMb().inverse()*rMtarget*coords->get_eeMt().inverse();

        mRobotInterface->getRobot()->move(romocc::MotionType::movep, new_bMee, 50, 25);
        moveToolManualButton->setText("Pause move");
    }
    mMovingToTarget = !mMovingToTarget;
}

void ApplicationGUI::moveToolToRegisteredTarget()
{
    if(mMovingToTarget)
    {
        moveToolRegisteredButton->setText("Move to registered target");
        mRobotInterface->getRobot()->stopMove(romocc::MotionType::stopj, 50);
    }else{
//        auto rMdata = mPreoperativeData->getSceneGraphNode()->getTransformation()->getTransform();
//        auto dataMtarget = Eigen::Affine3f::Identity();
//        Eigen::Matrix3f m;
//        m = Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitX())*
//            Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())*
//            Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
//
//        dataMtarget.translate(Eigen::Vector3f(200.0, 150.0, 200.0));
//        dataMtarget.linear() = dataMtarget.linear()*m;
//        auto rMtarget = rMdata*dataMtarget;
//        auto coords = mRobotInterface->getRobot()->getCoordinateSystem();
//
//        auto rMt = coords->get_rMb()*mRobotInterface->getCurrentState()->get_bMee()*coords->get_eeMt();
//        auto new_bMee = coords->get_rMb().inverse()*rMtarget.cast<double>()*coords->get_eeMt().inverse();
//        auto scaled_new_bMee = TransformUtils::Affine::scaleTranslation(new_bMee, 1.0/1000.0);
//
//        this->addCoordinateAxis(coords->get_rMb().cast<float>()*new_bMee.cast<float>());
//        this->addCoordinateAxis(coords->get_rMb().cast<float>()*new_bMee.cast<float>()*coords->get_eeMt().cast<float>());
//        this->renderCoordinateAxis(300);
//
//        auto target_joint_config = mRobotInterface->getCurrentState()->operationalConfigToJointConfig(scaled_new_bMee);
//        std::cout << target_joint_config << std::endl;

        mRobotInterface->getRobot()->move(romocc::MotionType::movej, mTargetJointConfig, 125, 60);
        moveToolRegisteredButton->setText("Pause move");
    }
    mMovingToTarget = !mMovingToTarget;
}

void ApplicationGUI::planMoveToRegisteredTarget() {
    auto rMdata = mPreoperativeData->getSceneGraphNode()->getTransformation()->getTransform();

    auto dataMtarget = mCalibrationWidget->getCalibrationTool()->get_registration_pcMt();

    std::cout << dataMtarget.matrix() << std::endl;
    auto rMtarget = rMdata*dataMtarget.cast<float>();
    auto coords = mRobotInterface->getRobot()->getCoordinateSystem();

    //auto rMt = coords->get_rMb()*mRobotInterface->getCurrentState()->get_bMee()*coords->get_eeMt();
    auto new_bMee = coords->get_rMb().inverse()*rMtarget.cast<double>()*coords->get_eeMt().inverse();
    mTargetOpConfig = new_bMee;
    auto scaled_new_bMee = TransformUtils::Affine::scaleTranslation(new_bMee, 1.0/1000.0);

    this->addCoordinateAxis(coords->get_rMb().cast<float>()*new_bMee.cast<float>());
    this->addCoordinateAxis(coords->get_rMb().cast<float>()*new_bMee.cast<float>()*coords->get_eeMt().cast<float>());

    mTargetJointConfig = mRobotInterface->getCurrentState()->operationalConfigToJointConfig(scaled_new_bMee);
    std::cout << mTargetJointConfig << std::endl;
    auto target_bMee = mRobotInterface->getCurrentState()->jointConfigToOperationalConfig(mTargetJointConfig);
    this->addCoordinateAxis(coords->get_rMb().cast<float>()*target_bMee.cast<float>());

    this->renderCoordinateAxis(300);
}

void ApplicationGUI::extractPointCloud() {
    stopComputationThread();
    //clearRenderVectors();

    if(mCameraPlayback)
    {
        std::cout << "Point cloud from playback not implemented." << std::endl;
    } else {
        //mCameraInterface->getProcessObject()->calculateTargetCloud(mCameraInterface->getStreamObject());
    }

    setupCameraVisualization();

    if(mRobotInterface->getRobot()->isConnected())
        setupRobotManipulatorVisualization();

    if(mUltrasoundStreaming)
        setupUltrasoundVisualization();

    //updateRenderers(mView3DRenderers, mView2DRenderers, mViewUSRenderers);

    startComputationThread();
}

// UI Setup

void ApplicationGUI::setupUI()
{
    View* view2D = createView();
    View* view3D = createView();
    View* viewUS = createView();

    setTitle("Example application");
    setWidth(1920);
    setHeight(1080);
    enableMaximized();

    view3D->set3DMode();
    view3D->setBackgroundColor(Color::White());
    view3D->setFixedWidth(720);

    view2D->set2DMode();
    view2D->setBackgroundColor(Color::White());
    view2D->setFixedWidth(540);

    viewUS->set2DMode();
    viewUS->setBackgroundColor(Color::White());
    viewUS->setFixedWidth(540);

    QVBoxLayout* menuLayout = new QVBoxLayout;
    menuLayout->setAlignment(Qt::AlignTop);
    int menuWidth = 540;

    // Title label
    QLabel* title = new QLabel;
    title->setText("<div style=\"text-align: center; font-weight: bold; font-size: 24px;\">Example application</div>");
    menuLayout->addWidget(title);

    mConnectionWidget = new ConnectionWidget(menuWidth);
    mConnectionWidget->addInterface(mRobotInterface);
    mConnectionWidget->addInterface(mCameraInterface);
    mConnectionWidget->addInterface(mUltrasoundInterface);
    menuLayout->addWidget(mConnectionWidget);

    mRobotMoveWidget = new RobotManualMoveWidget(mRobotInterface);

    mCalibrationWidget = new CalibrationWidget(menuWidth);
    mCalibrationWidget->addInterface(mRobotInterface);
    mCalibrationWidget->addInterface(mCameraInterface);
    mCalibrationWidget->addInterface(mUltrasoundInterface);

    QWidget *workflowWidget = getWorkflowWidget();
    workflowWidget->setFixedWidth(menuWidth);

    tabWidget = new QTabWidget;
    tabWidget->addTab(workflowWidget, "Workflow");
    tabWidget->addTab(mRobotMoveWidget, "Robot manual motion");
    tabWidget->addTab(mCalibrationWidget, "Calibration");
    tabWidget->setFixedWidth(menuWidth);
    menuLayout->addWidget(tabWidget);

    mRecordWidget = new RecordWidget(mCameraInterface, mUltrasoundInterface, menuWidth, 320);
    menuLayout->addWidget(mRecordWidget);
    menuLayout->addStretch();

    // Quit button
    QPushButton* quitButton = new QPushButton;
    quitButton->setText("Quit (q)");
    quitButton->setStyleSheet("QPushButton { background-color: red; color: white; }");
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

QWidget* ApplicationGUI::getWorkflowWidget()
{
    QWidget *group = new QWidget;

    QGridLayout *mainLayout = new QGridLayout();
    group->setLayout(mainLayout);

    int row = 0;
    calibrateButton = new QPushButton();
    mainLayout->addWidget(calibrateButton,row,0,1,2);
    calibrateButton->setText("Calibrate");
    calibrateButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    row++;
    QPushButton* loadPreopDataButton = new QPushButton;
    loadPreopDataButton->setText("Load preoperative data");
    QObject::connect(loadPreopDataButton, &QPushButton::clicked, std::bind(&ApplicationGUI::loadPreoperativeData, this));
    mainLayout->addWidget(loadPreopDataButton,row,0,1,2);

    row++;
    registerDataButton = new QPushButton();
    mainLayout->addWidget(registerDataButton,row,0,1,2);
    registerDataButton->setText("Register data");
    registerDataButton->setStyleSheet("QPushButton:checked { background-color: none; }");

//    registerTargetButton = new QPushButton();
//    mainLayout->addWidget(registerTargetButton,row,1,1,1);
//    registerTargetButton->setText("Register manual target");
//    registerTargetButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    row++;
    planMoveToolRegisteredButton = new QPushButton();
    mainLayout->addWidget(planMoveToolRegisteredButton, row, 0, 1, 2);
    planMoveToolRegisteredButton->setText("Plan move to registered target");
    planMoveToolRegisteredButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    row++;
    moveToolRegisteredButton = new QPushButton();
    mainLayout->addWidget(moveToolRegisteredButton,row,0,1,2);
    moveToolRegisteredButton->setText("Move to registered target");
    moveToolRegisteredButton->setStyleSheet("QPushButton:checked { background-color: none; }");

//    moveToolManualButton = new QPushButton();
//    mainLayout->addWidget(moveToolManualButton,row,1,1,1);
//    moveToolManualButton->setText("Move to manual target");
//    moveToolManualButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    row++;
    enableSegmentationButton = new QPushButton();
    mainLayout->addWidget(enableSegmentationButton, row, 0, 1, 2);
    enableSegmentationButton->setText("Enable segmentation");
    enableSegmentationButton->setStyleSheet("QPushButton:checked { background-color: none; }");


    return group;
}

//MouseListener::MouseListener(CameraInterface::pointer cameraInterface, View* view) : QObject(view) {
//    mCameraInterface = cameraInterface;
//    mPreviousMousePosition = Vector2i(-1, -1);
//    mView = view;
//}
//
//bool MouseListener::eventFilter(QObject *obj, QEvent *event) {
//    if(event->type() == QEvent::MouseButtonRelease) {
//        mPreviousMousePosition = Vector2i(-1, -1);
//    }
//    if(event->type() == QEvent::MouseMove) {
//        // Releay mouse movement to tracking
//        QMouseEvent *mouseEvent = static_cast<QMouseEvent *>(event);
//        // TODO need to go from view coordinates to physical to image coordinates
//        const Matrix4f perspectiveMatrix = mView->getPerspectiveMatrix();
//        const Matrix4f viewMatrix = mView->getViewMatrix();
//        Vector3f current(2.0f*(float)mouseEvent->x()/mView->width() - 1.0f, -(2.0f*(float)mouseEvent->y()/mView->height() -1.0f), 0);
//        current = (viewMatrix.inverse()*perspectiveMatrix.inverse()*current.homogeneous()).head(3);
//        if(mPreviousMousePosition.x() == -1 && mPreviousMousePosition.y() == -1) {
//            mPreviousMousePosition = current.cast<int>().head(2);
//        } else {
//            mCameraInterface->getProcessObject()->addLine(mPreviousMousePosition, current.cast<int>().head(2));
//            mPreviousMousePosition = current.cast<int>().head(2);
//        }
//    }
//
//    // standard event processing
//    return QObject::eventFilter(obj, event);
//}

void ApplicationGUI::loadPreoperativeData() {
    QFileDialog fileDialog(mWidget);
    fileDialog.setNameFilter("Data (*.mhd *.vtk)");
    QStringList filenames;
    if(fileDialog.exec()) {
        filenames = fileDialog.selectedFiles();
        for(QString qfilename : filenames) {
            std::string filename = qfilename.toUtf8().constData();

            QFileInfo fi(qfilename);
            QString extension = fi.suffix();

            stopComputationThread();
            removeAllRenderers();

            DataChannel::pointer port;

            if (extension.toStdString() == "mhd") {
                auto importer = ImageFileImporter::New();
                importer->setFilename(filename);

                auto resizer = ImageResizer::New();
                resizer->setInputConnection(importer->getOutputPort());
                resizer->setDepth(150);
                resizer->setWidth(150);
                resizer->setHeight(150);

                // Extract surface mesh using a threshold value
                auto extraction = SurfaceExtraction::New();
                extraction->setInputConnection(resizer->getOutputPort());
                extraction->setThreshold(300);
                port = extraction->getOutputPort();
                extraction->update();

                mPreoperativeData = port->getNextFrame<Mesh>();

                auto surfaceRenderer = TriangleRenderer::New();
                surfaceRenderer->setInputData(mPreoperativeData);
                getView(1)->addRenderer(surfaceRenderer);

            } else{
                auto importer = VTKMeshFileImporter::New();
                importer->setFilename(filename);
                port = importer->getOutputPort();
                importer->update();

                mPreoperativeData = port->getNextFrame<Mesh>();
                auto vertexRenderer = VertexRenderer::New();
                vertexRenderer->addInputData(mPreoperativeData, Color::Green(), 3.0);
                getView(1)->addRenderer(vertexRenderer);
            }

            if(mUltrasoundStreaming){
                mUltrasoundInterface->connect();
                setupUltrasoundVisualization();
            }

            if(mRobotInterface->getRobot()->isConnected()){
                mRobotInterface->disconnect();
                mRobotInterface->connect();
                setupRobotManipulatorVisualization();
            }

            if(mCameraStreaming || mCameraPlayback){
                mCameraInterface->connect();
                setupCameraVisualization();
                mConnectionWidget->updateCameraROI();
            }

            reinitializeViews();
            startComputationThread();
        }
    }
}

void ApplicationGUI::initializeRenderers()
{
    // Small hack to avoid transparent cad models in viewer
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

void ApplicationGUI::enableNeuralNetworkSegmentation() {
    stopComputationThread();
    removeAllRenderers();

    if(mRobotInterface->getRobot()->isConnected()){
        setupRobotManipulatorVisualization();
    }

    if(mCameraStreaming || mCameraPlayback){
        mCameraInterface->connect();
        setupCameraVisualization();
    }

    if(mUltrasoundStreaming){
        mUltrasoundInterface->connect();
        mUltrasoundInterface->enableSegmentation();
        setupUltrasoundVisualization();
    }

    reinitializeViews();
    startComputationThread();}
}