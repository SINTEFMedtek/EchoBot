#include "EchoBot/Tests/catch.hpp"
#include "EchoBot/Utilities/PointCloudUtilities.h"
#include "EchoBot/Exporters/PointCloudExporter.h"
#include "EchoBot/Core/Config.h"

#include "FAST/Visualization/SimpleWindow.hpp"
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
#include "FAST/Algorithms/NeuralNetwork/InferenceEngineManager.hpp"
#include <FAST/Algorithms/NeuralNetwork/SegmentationNetwork.hpp>
#include <FAST/Streamers/ImageFileStreamer.hpp>
#include <FAST/Algorithms/UltrasoundImageCropper/UltrasoundImageCropper.hpp>
#include <FAST/Streamers/MeshFileStreamer.hpp>


namespace echobot {

TEST_CASE("CT-PointCloud registration", "[EchoBot][Registration]") {
    auto registration_model_file = "/home/androst/dev/SINTEF/EchoBot/data/CT-Abdomen-surface-front.vtk";
    auto point_cloud_file = "/home/androst/EchoBot_Recordings/2020-01-06-114743 PhantomSimScan001/PointClouds/0.vtk";

    auto importer = VTKMeshFileImporter::New();
    importer->setFilename(registration_model_file);

    auto port = importer->getOutputPort();
    importer->update();
    auto registrationCloud = port->getNextFrame<Mesh>();

    auto pc_importer = VTKMeshFileImporter::New();
    pc_importer->setFilename(point_cloud_file);

    auto meshProcessor = MeshProcessing::New();
    meshProcessor->setInputConnection(pc_importer->getOutputPort());
    meshProcessor->setBounds(-600, 300, -700, 200, 1000, 1680);

    auto pc_port = meshProcessor->getOutputPort();
    meshProcessor->update();
    auto currentCloud = pc_port->getNextFrame<Mesh>();

    // Modify point clouds
    auto regCloudReduced = decimateMesh(registrationCloud, (double) 2500.0 / registrationCloud->getNrOfVertices());
    auto currentCloudReduced = decimateMesh(currentCloud, (double) 2500.0 / currentCloud->getNrOfVertices());

    // Set registration settings
    float uniformWeight = 0.5;
    double tolerance = 1e-3;

    std::cout << regCloudReduced->getSceneGraphNode()->getTransformation()->getTransform().matrix() << "\n"
              << std::endl;
    std::cout << currentCloudReduced->getSceneGraphNode()->getTransformation()->getTransform().matrix() << "\n"
              << std::endl;

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

    auto regCloudRenderer = VertexRenderer::New();
    regCloudRenderer->setInputData(regCloudReduced);

    auto cloudRenderer = VertexRenderer::New();
    cloudRenderer->addInputData(currentCloud);
    cloudRenderer->addInputData(mesh);

    auto window = fast::SimpleWindow::New();
    window->setTimeout(20000);
    window->set3DMode();
    window->addRenderer(regCloudRenderer);
    window->addRenderer(cloudRenderer);
    window->start();
}

TEST_CASE("Test segmentation", "[EchoBot][NeuralNetwork]") {
    auto streamer = ImageFileStreamer::New();
    streamer->setFilenameFormat("/home/androst/dev/FAST/data/US/JugularVein/US-2D_#.mhd");
    streamer->enableLooping();
    auto inputStream = streamer;

    auto segmentation = SegmentationNetwork::New();
    segmentation->setScaleFactor(1.0f / 255.0f);
    segmentation->setInferenceEngine("TensorFlowCUDA");
    const auto engine = segmentation->getInferenceEngine()->getName();
    segmentation->setOutputNode(0, "conv2d_23/truediv");

    segmentation->load("/home/androst/dev/FAST/data/NeuralNetworkModels/jugular_vein_segmentation.pb");
    segmentation->setInputConnection(inputStream->getOutputPort());
    segmentation->enableRuntimeMeasurements();

    auto segmentationRenderer = SegmentationRenderer::New();
    segmentationRenderer->addInputConnection(segmentation->getOutputPort());
    segmentationRenderer->setOpacity(0.25);
    segmentationRenderer->setColor(fast::Segmentation::LABEL_FOREGROUND, Color::Red());
    segmentationRenderer->setColor(fast::Segmentation::LABEL_BLOOD, Color::Blue());

    auto imageRenderer = ImageRenderer::New();
    imageRenderer->setInputConnection(inputStream->getOutputPort());

    auto window = fast::SimpleWindow::New();
    window->addRenderer(imageRenderer);
    window->addRenderer(segmentationRenderer);
    window->set2DMode();
    window->getView()->setBackgroundColor(Color::Black());
    window->setTimeout(10000);
    window->start();
    segmentation->getAllRuntimes()->printAll();
}

TEST_CASE("Accumulate point cloud") {
    bool dumpFile = false;
    auto filepath = Config::getTestDataPath() + "phantom_scan/PointClouds/#.vtk";

    auto meshStreamer = MeshFileStreamer::New();
    meshStreamer->setFilenameFormat(filepath);
    meshStreamer->enableLooping();
    meshStreamer->setSleepTime(33.3);

    auto meshProcessor = MeshProcessing::New();
    meshProcessor->setInputConnection(meshStreamer->getOutputPort());
    meshProcessor->setBounds(-600, 300, -700, 200, 1000, 1680);

    auto port = meshProcessor->getOutputPort();

    std::vector<MeshVertex> accumulatedPoints;
    for(int i = 0; i < 5; i++){
        meshProcessor->update();
        auto mesh = port->getNextFrame<Mesh>();
        auto meshAccess = mesh->getMeshAccess(ACCESS_READ);
        auto vertices =  meshAccess->getVertices();
        accumulatedPoints.insert(accumulatedPoints.end(), vertices.begin(), vertices.end());
        std::cout << accumulatedPoints.size() << std::endl;
    }
    auto accumulatedMesh = Mesh::New();
    accumulatedMesh->create(accumulatedPoints);

    Vector3f centroid = calculateCentroid(accumulatedPoints);
    Eigen::Affine3f refMcentroid = Eigen::Affine3f::Identity();
    refMcentroid.translate(centroid);

    std::cout << refMcentroid.inverse().matrix() << std::endl;

    AffineTransformation::pointer transform = AffineTransformation::New();
    transform->setTransform(refMcentroid.inverse());
    accumulatedMesh->getSceneGraphNode()->setTransformation(transform);

    auto pcRenderer = VertexRenderer::New();
    pcRenderer->addInputData(accumulatedMesh);
    pcRenderer->setDefaultSize(1.5);

    if(dumpFile){
        auto meshExporter = PointCloudExporter::New();
        meshExporter->setInputData(accumulatedMesh);
        meshExporter->setWriteNormals(false);
        meshExporter->setWriteColors(true);
        meshExporter->setFilename("accumulated_phantom_scan_five.vtk");
        meshExporter->update();
    }

    auto window = fast::SimpleWindow::New();
    window->addRenderer(pcRenderer);
    window->setTimeout(20000);
    window->start();
}

TEST_CASE("PhantomPC-PointCloud registration", "[EchoBot][Registration]") {
    auto registration_model_file = Config::getRegistrationDataPath() + "accumulated_pc_phantom.vtk";
    auto point_cloud_file = Config::getTestDataPath() + "phantom_scan/PointClouds/100.vtk";

    auto importer = VTKMeshFileImporter::New();
    importer->setFilename(registration_model_file);

    auto port = importer->getOutputPort();
    importer->update();
    auto registrationCloud = port->getNextFrame<Mesh>();

    auto pc_importer = VTKMeshFileImporter::New();
    pc_importer->setFilename(point_cloud_file);

    auto meshProcessor = MeshProcessing::New();
    meshProcessor->setInputConnection(pc_importer->getOutputPort());
    meshProcessor->setBounds(-600, 300, -700, 200, 1000, 1680);

    auto pc_port = meshProcessor->getOutputPort();
    meshProcessor->update();
    auto currentCloud = pc_port->getNextFrame<Mesh>();

    // Modify point clouds
    auto regCloudReduced = decimateMesh(registrationCloud, (double) 2500.0 / registrationCloud->getNrOfVertices());
    auto currentCloudReduced = decimateMesh(currentCloud, (double) 2500.0 / currentCloud->getNrOfVertices());

    // Set registration settings
    float uniformWeight = 0.5;
    double tolerance = 1e-3;

    std::cout << regCloudReduced->getSceneGraphNode()->getTransformation()->getTransform().matrix() << "\n"
              << std::endl;
    std::cout << currentCloudReduced->getSceneGraphNode()->getTransformation()->getTransform().matrix() << "\n"
              << std::endl;

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

    auto regCloudRenderer = VertexRenderer::New();
    regCloudRenderer->setInputData(regCloudReduced);

    auto cloudRenderer = VertexRenderer::New();
    cloudRenderer->addInputData(currentCloud, Color::Cyan(), 1.5);
    cloudRenderer->addInputData(mesh);

    auto window = fast::SimpleWindow::New();
    window->setTimeout(20000);
    window->set3DMode();
    window->addRenderer(regCloudRenderer);
    window->addRenderer(cloudRenderer);
    window->start();
}

TEST_CASE("Neural network processing", "[EchoBot][NeuralNetwork]") {
        auto filepath = "/media/Data/UltrasoundData/2019_ROMO/Interim/Annotationweb/Phantom/Scan001/Image-2D_#.mhd";

        auto streamer = ImageFileStreamer::New();
        streamer->setFilenameFormat(filepath);
        streamer->setSleepTime(25);
        streamer->enableLooping();

        auto cropper = fast::UltrasoundImageCropper::New();
        cropper->setInputConnection(streamer->getOutputPort());

        auto segmentation = SegmentationNetwork::New();
        segmentation->setScaleFactor(1.0f / 255.0f);
        const auto engine = segmentation->getInferenceEngine()->getName();
        if(engine.substr(0,10) == "TensorFlow") {
            // TensorFlow needs to know what the output node is called
            segmentation->setOutputNode(0, "conv2d_23/truediv");
        }

        segmentation->load(fast::join(Config::getNeuralNetworkModelPath(), "aorta_segmentation_new.pb"));
        segmentation->setInputConnection(cropper->getOutputPort());
        segmentation->enableRuntimeMeasurements();

        auto segmentationRenderer = SegmentationRenderer::New();
        segmentationRenderer->addInputConnection(segmentation->getOutputPort());
        segmentationRenderer->setOpacity(0.25);
        segmentationRenderer->setColor(fast::Segmentation::LABEL_FOREGROUND, Color::Red());
        segmentationRenderer->setColor(fast::Segmentation::LABEL_BLOOD, Color::Black());

        auto imageRenderer = ImageRenderer::New();
        imageRenderer->setInputConnection(cropper->getOutputPort());

        auto window = fast::SimpleWindow::New();
        window->addRenderer(imageRenderer);
        window->addRenderer(segmentationRenderer);
        window->set2DMode();
        window->setTimeout(25000);
        window->getView()->setBackgroundColor(Color::Black());
        window->start();
        segmentation->getAllRuntimes()->printAll();
}

}