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
#include <QFileDialog>
#include <Eigen/Dense>

#include "FAST/Visualization/MultiViewWindow.hpp"
#include <FAST/Visualization/SegmentationRenderer/SegmentationRenderer.hpp>
#include <FAST/Visualization/VertexRenderer/VertexRenderer.hpp>
#include <FAST/Visualization/LineRenderer/LineRenderer.hpp>
#include <FAST/Streamers/MeshFileStreamer.hpp>
#include <FAST/Streamers/ImageFileStreamer.hpp>
#include <FAST/Exporters/VTKMeshFileExporter.hpp>
#include <FAST/Importers/VTKMeshFileImporter.hpp>
#include "FAST/Visualization/TriangleRenderer/TriangleRenderer.hpp"
#include "FAST/Streamers/RealSenseStreamer.hpp"
#include "FAST/Visualization/ImageRenderer/ImageRenderer.hpp"
#include "FAST/Importers/ImageFileImporter.hpp"
#include "FAST/Algorithms/SurfaceExtraction/SurfaceExtraction.hpp"
#include "FAST/Algorithms/ImageResizer/ImageResizer.hpp"
#include "FAST/Algorithms/CoherentPointDrift/CoherentPointDrift.hpp"
#include "FAST/Algorithms/CoherentPointDrift/Rigid.hpp"
#include "FAST/Algorithms/SegmentationVolumeReconstructor/SegmentationVolumeReconstructor.hpp"


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

    QObject::connect(mCameraMinDepthLineEdit, &QLineEdit::textChanged, std::bind(&ApplicationGUI::updateCameraROI, this));
    QObject::connect(mCameraMaxDepthLineEdit, &QLineEdit::textChanged, std::bind(&ApplicationGUI::updateCameraROI, this));
    QObject::connect(mCameraMinWidthLineEdit, &QLineEdit::textChanged, std::bind(&ApplicationGUI::updateCameraROI, this));
    QObject::connect(mCameraMaxWidthLineEdit, &QLineEdit::textChanged, std::bind(&ApplicationGUI::updateCameraROI, this));
    QObject::connect(mCameraMinHeightLineEdit, &QLineEdit::textChanged, std::bind(&ApplicationGUI::updateCameraROI, this));
    QObject::connect(mCameraMaxHeightLineEdit, &QLineEdit::textChanged, std::bind(&ApplicationGUI::updateCameraROI, this));

    QObject::connect(usConnectButton, &QPushButton::clicked, std::bind(&ApplicationGUI::connectToUltrasound, this));

    QObject::connect(calibrateButton, &QPushButton::clicked, std::bind(&ApplicationGUI::calibrateSystem, this));
    QObject::connect(registerDataButton, &QPushButton::clicked, std::bind(&ApplicationGUI::registerCloudToData, this));
    QObject::connect(registerTargetButton, &QPushButton::clicked, std::bind(&ApplicationGUI::registerTarget, this));
    QObject::connect(moveToolManualButton, &QPushButton::clicked, std::bind(&ApplicationGUI::moveToolToManualTarget, this));
    QObject::connect(moveToolRegisteredButton, &QPushButton::clicked, std::bind(&ApplicationGUI::moveToolToRegisteredTarget, this));
}


// Robot

void ApplicationGUI::robotConnectButtonSlot()
{
    mRobotInterface->robot.configure(corah::Manipulator::UR5,mRobotIPLineEdit->text(),30003);
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

    if(mCameraStreaming || mCameraPlayback)
        setupCameraVisualization();

    if(mUltrasoundStreaming)
        setupUltrasoundVisualization();

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

    mCameraStreamer = RealSenseStreamer::New();
    mCameraStreamer->getReporter().setReportMethod(Reporter::COUT);
    //mCameraStreamer->setPointCloudFiltering(true);

    // Tracking
    mCameraInterface = CameraInterface::New();

    clearRenderVectors();
    setupCameraVisualization();

    if(mRobotInterface->robot.isConnected())
        setupRobotManipulatorVisualization();

    if(mUltrasoundStreaming)
        setupUltrasoundVisualization();

    updateRenderers(mView3DRenderers, mView2DRenderers, mViewUSRenderers);
    getView(0)->installEventFilter(new MouseListener(mCameraInterface, getView(0)));

    mCameraStreaming = true;

    startComputationThread();
}

void ApplicationGUI::setupCameraVisualization() {

    if(mCameraPlayback){
        for(auto it: mCameraPlaybackStreamers){
            mCameraInterface->setInputConnection(it.first, it.second->getOutputPort(0));
        }
    }
    else{
        mCameraInterface->setInputConnection(0, mCameraStreamer->getOutputPort(0));
        mCameraInterface->setInputConnection(1, mCameraStreamer->getOutputPort(2));
    }

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

    // Render target cloud if exists
    if(mCameraInterface->isTargetCloudExtracted()){
        VertexRenderer::pointer targetCloudRenderer = VertexRenderer::New();
        targetCloudRenderer->addInputConnection(mCameraInterface->getOutputPort(3));
        targetCloudRenderer->setDefaultSize(1.5);
        targetCloudRenderer->setDefaultColor(Color::Green());
        mView3DRenderers.push_back(targetCloudRenderer);
    } else {
        mView2DRenderers.push_back(annotationRenderer);
    }
}

void ApplicationGUI::updateRenderers(std::vector<Renderer::pointer> mView3DRenderers,
                                     std::vector<Renderer::pointer> mView2DRenderers,
                                     std::vector<Renderer::pointer> mViewUSRenderers)
{
    getView(0)->removeAllRenderers();
    getView(0)->set2DMode();
    getView(0)->setBackgroundColor(Color::White());
    for(auto it: mView2DRenderers) {
        std::cout << "2D: " << it->getNameOfClass() << std::endl;
        getView(0)->addRenderer(it);
    }
    getView(0)->reinitialize();

    getView(1)->removeAllRenderers();
    getView(1)->set3DMode();
    getView(1)->setBackgroundColor(Color::White());

    for(auto it: mView3DRenderers){
        std::cout << "3D: " << it->getNameOfClass() << std::endl;
        getView(1)->addRenderer(it);
    }

    getView(1)->reinitialize();

    getView(2)->removeAllRenderers();
    getView(2)->set2DMode();
    getView(2)->setBackgroundColor(Color::White());

    for(auto it: mViewUSRenderers){
        std::cout << "US: " << it->getNameOfClass() << std::endl;
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
    mCameraStreamer->setMinRange(mCameraMinDepthLineEdit->text().toFloat());
    mCameraStreamer->setMaxRange(mCameraMaxDepthLineEdit->text().toFloat());
    mCameraStreamer->setMinWidth(mCameraMinWidthLineEdit->text().toFloat());
    mCameraStreamer->setMaxWidth(mCameraMaxWidthLineEdit->text().toFloat());
    mCameraStreamer->setMinHeight(mCameraMinHeightLineEdit->text().toFloat());
    mCameraStreamer->setMaxHeight(mCameraMaxHeightLineEdit->text().toFloat());
}

void ApplicationGUI::restartCamera() {

    stopComputationThread();
    clearRenderVectors();

    setupCameraVisualization();

    if(mRobotInterface->robot.isConnected())
        setupRobotManipulatorVisualization();

    if(mUltrasoundStreaming)
        setupUltrasoundVisualization();

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

    mUltrasoundStreamer = IGTLinkStreamer::New();
    mUltrasoundStreamer->setConnectionAddress(mUsIPLineEdit->text().toStdString());
    mUltrasoundStreamer->setConnectionPort(18944);

    mUltrasoundInterface = UltrasoundInterface::New();
    mUltrasoundInterface->setInputConnection(mUltrasoundStreamer->getOutputPort());

    stopComputationThread();
    clearRenderVectors();

    setupUltrasoundVisualization();

    if(mRobotInterface->robot.isConnected())
        setupRobotManipulatorVisualization();
        mUltrasoundInterface->setRobotInterface(mRobotInterface);

    if(mCameraStreaming || mCameraPlayback)
        setupCameraVisualization();

    updateRenderers(mView3DRenderers, mView2DRenderers, mViewUSRenderers);

    mUltrasoundStreaming = true;
    startComputationThread();
}

void ApplicationGUI::setupUltrasoundVisualization()
{
    if(mUltrasoundStreaming) {
        mUltrasoundStreamer->stop();
        mUltrasoundStreamer->stopPipeline();
        mUltrasoundInterface->stopPipeline();

        mUltrasoundStreamer = IGTLinkStreamer::New();
        mUltrasoundStreamer->setConnectionAddress(mUsIPLineEdit->text().toStdString());
        mUltrasoundStreamer->setConnectionPort(18944);

        mUltrasoundInterface = UltrasoundInterface::New();
        mUltrasoundInterface->setInputConnection(mUltrasoundStreamer->getOutputPort());

        if(mRobotInterface->robot.isConnected())
            mUltrasoundInterface->setRobotInterface(mRobotInterface);
    }

    ImageRenderer::pointer usRenderer = ImageRenderer::New();
    usRenderer->addInputConnection(mUltrasoundInterface->getOutputPort(0));

    SegmentationRenderer::pointer segmentationRenderer = SegmentationRenderer::New();
    segmentationRenderer->addInputConnection(mUltrasoundInterface->getOutputPort(1));

    SegmentationVolumeReconstructor::pointer reconstructor = SegmentationVolumeReconstructor::New();
    reconstructor->setInputConnection(mUltrasoundInterface->getOutputPort(1));

    SurfaceExtraction::pointer extraction = SurfaceExtraction::New();
    extraction->setInputConnection(reconstructor->getOutputPort());

    TriangleRenderer::pointer surfaceRenderer = TriangleRenderer::New();
    surfaceRenderer->addInputConnection(extraction->getOutputPort());

    mViewUSRenderers.push_back(usRenderer);
    mView3DRenderers.push_back(usRenderer);

    mViewUSRenderers.push_back(segmentationRenderer);
    mView3DRenderers.push_back(surfaceRenderer);
}

// Calibration
void ApplicationGUI::calibrateSystem()
{
    Eigen::Affine3d rMb = Eigen::Affine3d::Identity();

    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(0.51*M_PI, Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(0.96*M_PI, Eigen::Vector3d::UnitX())*
        Eigen::AngleAxisd(0.0*M_PI, Eigen::Vector3d::UnitY());

    rMb.translate(Eigen::Vector3d(-800,150,950)); // -500, 370, 1000 (y,x,z)
    rMb.linear() = rMb.linear()*m;

    mRobotInterface->robot.set_rMb(rMb);

    Eigen::Affine3d eeMt = Eigen::Affine3d::Identity();

    Eigen::Matrix3d rotProbe;
    rotProbe = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())*
               Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())*
               Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());

    eeMt.translate(Eigen::Vector3d(-100,0,100));
    eeMt.linear() = eeMt.linear()*rotProbe;

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

void ApplicationGUI::registerCloudToData()
{
    auto registration_model_file = "/home/androst/dev/ROMO/FAST-ROMO/data/CT-Abdomen-surface-front.vtk";

    auto importer = VTKMeshFileImporter::New();
    importer->setFilename(registration_model_file);

    auto port = importer->getOutputPort();
    importer->update(0);
    auto registrationCloud = port->getNextFrame<Mesh>();

    DataPort::pointer streamPort;
    if(mCameraStreaming){
        streamPort = mCameraStreamer->getOutputPort(2);
    } else{
        streamPort = mCameraPlaybackStreamers[1]->getOutputPort();
    }
    auto currentCloud = streamPort->getNextFrame<Mesh>();

    // Modify point clouds
    auto regCloudReduced = mCameraInterface->createReducedSample(registrationCloud, (double)2500.0/registrationCloud->getNrOfVertices());
    //auto regCloudReduced = mCameraInterface->createReducedSample(mPreoperativeData, (double)2500.0/mPreoperativeData->getNrOfVertices());
    auto currentCloudReduced = mCameraInterface->createReducedSample(currentCloud, (double)2500.0/currentCloud->getNrOfVertices());

    // Set registration settings
    float uniformWeight = 0.5;
    double tolerance = 1e-3;

    std::cout << regCloudReduced->getSceneGraphNode()->getTransformation()->getTransform().matrix()  << "\n" << std::endl;
    std::cout << currentCloudReduced->getSceneGraphNode()->getTransformation()->getTransform().matrix()  << "\n" << std::endl;

    // Run Coherent Point Drift
    auto cpd = CoherentPointDriftRigid::New();
    cpd->setFixedMesh(regCloudReduced);
    cpd->setMovingMesh(currentCloudReduced);
    cpd->setMaximumIterations(50);
    cpd->setTolerance(tolerance);
    cpd->setUniformWeight(uniformWeight);

    auto cpdPort = cpd->getOutputPort();
    cpd->update(0);
    Mesh::pointer mesh = cpdPort->getNextFrame<Mesh>();
    //mPreoperativeData = mesh;

    Eigen::Affine3f eigtransform = mesh->getSceneGraphNode()->getTransformation()->getTransform().inverse();
    Eigen::Affine3f offset = Eigen::Affine3f::Identity();

    Eigen::Matrix3f m;
    m = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ())*
        Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())*
        Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY());

    offset.translate(Eigen::Vector3f(163,302,-570));
    offset.linear() = offset.linear()*m;

    std::cout << eigtransform.matrix()  << "\n" << std::endl;

    AffineTransformation::pointer transform = AffineTransformation::New();
    transform->setTransform(eigtransform*offset);

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

        mRobotInterface->robot.move(corah::MotionType::movep, new_bMee, 50, 25);
        moveToolManualButton->setText("Abort move");
    }
    mMovingToTarget = !mMovingToTarget;
}

void ApplicationGUI::moveToolToRegisteredTarget()
{
    if(mMovingToTarget)
    {
        moveToolRegisteredButton->setText("Move to registered target");
        mRobotInterface->robot.stopMove(corah::MotionType::stopj, 50);
    }else{
        Eigen::Affine3f rMdata = mPreoperativeData->getSceneGraphNode()->getTransformation()->getTransform();
        Eigen::Affine3d dataMtarget;
//        dataMtarget.matrix() <<  0.08,   0.04,  -0.89, 160.6,
//                                 0.99,   1.  ,   0.01,  88.2 ,
//                                 0.13,   0.06,   0.46, 216.0,
//                                 0.  ,   0.  ,   0.  ,   1.;

        dataMtarget.matrix() <<  -1.,   0.,  0., 160.6,
                                 0.,   0.,  1.,  88.2,
                                 0.,  1., 0., 216.0,
                                 0.,   0.,  0.,   1.;


        Eigen::Affine3d rMtarget = rMdata.cast<double>()*dataMtarget;
        Eigen::Affine3d new_bMee = mRobotInterface->robot.get_rMb().inverse()*rMtarget*mRobotInterface->robot.get_eeMt().inverse();

        std::cout << new_bMee.matrix() << std::endl;

        mRobotInterface->robot.move(corah::MotionType::movep, new_bMee, 50, 25);
        moveToolManualButton->setText("Abort move");

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

        // Create recording pathkom
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
    mCameraPlayback = !mCameraPlayback;
    if(!mCameraPlayback) {
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

        if(mCameraStreaming)
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
        MeshFileStreamer::pointer meshStreamer = MeshFileStreamer::New();
        meshStreamer->setFilenameFormat(selectedRecordingPointClouds + "#.vtk");
        meshStreamer->enableLooping();
        meshStreamer->update(0);

        ImageFileStreamer::pointer imageStreamer = ImageFileStreamer::New();
        imageStreamer->setFilenameFormat(selectedRecordingImages + "Cam-2D_#.mhd");
        imageStreamer->enableLooping();
        imageStreamer->setSleepTime(100);
        imageStreamer->update(0);

        mCameraInterface = CameraInterface::New();
        mCameraInterface->setInputConnection(0, imageStreamer->getOutputPort());
        mCameraInterface->setInputConnection(1, meshStreamer->getOutputPort());

        mCameraPlaybackStreamers[0] = imageStreamer;
        mCameraPlaybackStreamers[1] = meshStreamer;

        stopComputationThread();
        clearRenderVectors();

        setupCameraVisualization();
        if(mRobotInterface->robot.isConnected())
            setupRobotManipulatorVisualization();

        updateRenderers(mView3DRenderers, mView2DRenderers, mViewUSRenderers);
        getView(0)->installEventFilter(new MouseListener(mCameraInterface, getView(0)));

        startComputationThread();

        mPlayButton->setText("Stop");
        mPlayButton->setStyleSheet("QPushButton { background-color: red; color: white; }");
    }
}


void ApplicationGUI::extractPointCloud() {
    stopComputationThread();
    clearRenderVectors();

    if(mCameraPlayback)
    {
        std::cout << "Point cloud from playback not implemented." << std::endl;
    } else {
        mCameraInterface->calculateTargetCloud(mCameraStreamer);
    }

    setupCameraVisualization();

    if(mRobotInterface->robot.isConnected())
        setupRobotManipulatorVisualization();

    if(mUltrasoundStreaming)
        setupUltrasoundVisualization();

    updateRenderers(mView3DRenderers, mView2DRenderers, mViewUSRenderers);

    startComputationThread();
}


// UI Setup

void ApplicationGUI::setupUI()
{
    View* view2D = createView();
    View* view3D = createView();
    View* viewUS = createView();

    const int menuWidth = 520;

    setTitle("Aortic Aneurysm Exam");
    setWidth(1920);
    setHeight(1080);
    enableMaximized();

    view3D->set3DMode();
    view3D->setBackgroundColor(Color::White());
    view3D->setFixedWidth(760);

    view2D->set2DMode();
    view2D->setBackgroundColor(Color::White());
    view2D->setFixedWidth(580);

    viewUS->set2DMode();
    viewUS->setBackgroundColor(Color::White());
    viewUS->setFixedWidth(580);

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
    connectionsTabWidget->setFixedWidth(menuWidth);
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

    mCameraMinDepthLineEdit = new QLineEdit();
    mCameraMaxDepthLineEdit = new QLineEdit();
    mCameraMinDepthLineEdit->setText(QString("0"));
    mCameraMaxDepthLineEdit->setText(QString("2000"));

    mCameraMinWidthLineEdit = new QLineEdit();
    mCameraMaxWidthLineEdit = new QLineEdit();
    mCameraMinWidthLineEdit->setText(QString("-1000"));
    mCameraMaxWidthLineEdit->setText(QString("1000"));

    mCameraMinHeightLineEdit = new QLineEdit();
    mCameraMaxHeightLineEdit = new QLineEdit();
    mCameraMinHeightLineEdit->setText(QString("-1000"));
    mCameraMaxHeightLineEdit->setText(QString("1000"));

    mainLayout->addWidget(new QLabel("Depth range [mm]: "), 0, 0, 1, 1);
    mainLayout->addWidget(mCameraMinDepthLineEdit,0,1,1,1);
    mainLayout->addWidget(mCameraMaxDepthLineEdit,0,2,1,1);

    mainLayout->addWidget(new QLabel("Width range [mm]: "), 1, 0, 1, 1);
    mainLayout->addWidget(mCameraMinWidthLineEdit,1,1,1,1);
    mainLayout->addWidget(mCameraMaxWidthLineEdit,1,2,1,1);

    mainLayout->addWidget(new QLabel("Height range [mm]: "), 2, 0, 1, 1);
    mainLayout->addWidget(mCameraMinHeightLineEdit,2,1,1,1);
    mainLayout->addWidget(mCameraMaxHeightLineEdit,2,2,1,1);

    cameraConnectButton = new QPushButton();

    QIcon icon;
    icon.addFile(mGraphicsFolderName+"network-idle.ico", QSize(), QIcon::Normal, QIcon::Off);
    icon.addFile(mGraphicsFolderName+"network-transmit-receive.ico", QSize(), QIcon::Normal, QIcon::On);
    cameraConnectButton->setIcon(icon);
    cameraConnectButton->setToolTip("Connect to robot");
    cameraConnectButton->setText("Connect");
    cameraConnectButton->setCheckable(true);
    cameraConnectButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    mainLayout->addWidget(cameraConnectButton,0,3,1,1);

    cameraDisconnectButton = new QPushButton(QIcon(mGraphicsFolderName+"network-offline.ico"),"Disconnect");
    mainLayout->addWidget(cameraDisconnectButton,2,3,1,1);

    return group;
}

QWidget* ApplicationGUI::getRobotConnectionWidget()
{
    QWidget *group = new QWidget;
    QGridLayout *mainLayout = new QGridLayout();
    group->setLayout(mainLayout);

    int row = 0;
    mRobotIPLineEdit = new QLineEdit();
    robotConnectButton = new QPushButton();
    mainLayout->addWidget(new QLabel("IP Address: "), row, 0, 1, 1);
    mainLayout->addWidget(mRobotIPLineEdit, row, 1,1,1);
    mainLayout->addWidget(robotConnectButton,row,2,1,1);

    mRobotIPLineEdit->setText("10.218.140.123"); // 10.218.140.114
    mRobotIPLineEdit->setAlignment(Qt::AlignCenter);

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
    mUsIPLineEdit = new QLineEdit();
    usConnectButton = new QPushButton();
    mainLayout->addWidget(new QLabel("IP Address: "), row, 0, 1, 1);
    mainLayout->addWidget(mUsIPLineEdit, row, 1,1,1);
    mainLayout->addWidget(usConnectButton,row,2,1,1);

    mUsIPLineEdit->setText("192.168.140.116"); // 10.218.140.114
    mUsIPLineEdit->setAlignment(Qt::AlignCenter);

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
    mainLayout->addWidget(registerDataButton,row,0,1,1);
    registerDataButton->setText("Register data");
    registerDataButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    registerTargetButton = new QPushButton();
    mainLayout->addWidget(registerTargetButton,row,1,1,1);
    registerTargetButton->setText("Register manual target");
    registerTargetButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    row++;
    moveToolManualButton = new QPushButton();
    mainLayout->addWidget(moveToolManualButton,row,1,1,1);
    moveToolManualButton->setText("Move to manual target");
    moveToolManualButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    moveToolRegisteredButton = new QPushButton();
    mainLayout->addWidget(moveToolRegisteredButton,row,0,1,1);
    moveToolRegisteredButton->setText("Move to registered target");
    moveToolRegisteredButton->setStyleSheet("QPushButton:checked { background-color: none; }");

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
            clearRenderVectors();

            DataPort::pointer port;

            if (extension.toStdString() == "mhd") {
                ImageFileImporter::pointer importer = ImageFileImporter::New();
                importer->setFilename(filename);

                ImageResizer::pointer resizer = ImageResizer::New();
                resizer->setInputConnection(importer->getOutputPort());
                resizer->setDepth(150);
                resizer->setWidth(150);
                resizer->setHeight(150);

                // Extract surface mesh using a threshold value
                SurfaceExtraction::pointer extraction = SurfaceExtraction::New();
                extraction->setInputConnection(resizer->getOutputPort());
                extraction->setThreshold(300);
                port = extraction->getOutputPort();
                extraction->update(0);

                mPreoperativeData = port->getNextFrame<Mesh>();

                auto surfaceRenderer = TriangleRenderer::New();
                surfaceRenderer->setInputData(mPreoperativeData);
                mView3DRenderers.push_back(surfaceRenderer);

            } else{
                auto importer = VTKMeshFileImporter::New();
                importer->setFilename(filename);
                port = importer->getOutputPort();
                importer->update(0);

                mPreoperativeData = port->getNextFrame<Mesh>();
                auto vertexRenderer = VertexRenderer::New();
                vertexRenderer->addInputData(mPreoperativeData, Color::Green(), 3.0);
                mView3DRenderers.push_back(vertexRenderer);
            }

            if(mUltrasoundStreaming)
                setupUltrasoundVisualization();

            if(mRobotInterface->robot.isConnected())
                setupRobotManipulatorVisualization();

            if(mCameraStreaming || mCameraPlayback)
                setupCameraVisualization();

            updateRenderers(mView3DRenderers, mView2DRenderers, mViewUSRenderers);
            startComputationThread();
        }
    }
}

LineRenderer::pointer ApplicationGUI::createCoordinateFrameRenderer(Eigen::Affine3f transform)
{
    Mesh::pointer mesh = Mesh::New();

//    std::vector<MeshVertex> vertices = {
//            MeshVertex(transform.translation()),
//            MeshVertex(transform.linear().col(0)*100),
//            MeshVertex(transform.linear().col(1)*100),
//            MeshVertex(transform.linear().col(2)*100),
//    };

    std::vector<MeshVertex> vertices = {
            MeshVertex(Vector3f(0, 0, 0)),
            MeshVertex(Vector3f(500, 0, 0)),
            MeshVertex(Vector3f(0, 500, 0)),
            MeshVertex(Vector3f(0, 0, 500)),
    };


    std::vector<MeshLine> lines = {
            MeshLine(0, 1),
            MeshLine(0, 2),
            MeshLine(0, 3),
    };
    mesh->create(vertices, lines);

    LineRenderer::pointer lineRenderer = LineRenderer::New();
    lineRenderer->addInputData(mesh);
    lineRenderer->setDefaultLineWidth(100);
    lineRenderer->setColor(0, Color::Red());

    return lineRenderer;
}

}