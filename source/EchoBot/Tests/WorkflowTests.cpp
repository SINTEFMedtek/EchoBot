//
// Created by androst on 06.01.20.
//

#include "EchoBot/Tests/catch.hpp"
#include "EchoBot/Utilities/PointCloudUtilities.h"

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

namespace echobot
{
using namespace fast;

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

    auto pc_port = pc_importer->getOutputPort();
    pc_importer->update();
    auto currentCloud = pc_port->getNextFrame<Mesh>();
    currentCloud = reduceMeshExtent(currentCloud, 1000, 1700, -600, 300, -1000, 1000);

    // Modify point clouds
    auto regCloudReduced = decimateMesh(registrationCloud, (double)2500.0/registrationCloud->getNrOfVertices());
    auto currentCloudReduced = decimateMesh(currentCloud, (double)2500.0/currentCloud->getNrOfVertices());

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
    cpd->update();
    Mesh::pointer mesh = cpdPort->getNextFrame<Mesh>();

    auto regCloudRenderer = VertexRenderer::New();
    regCloudRenderer->setInputData(regCloudReduced);

    auto cloudRenderer = VertexRenderer::New();
    cloudRenderer->addInputData(currentCloud);
    cloudRenderer->addInputData(mesh);

    SimpleWindow::pointer window = SimpleWindow::New();
    window->setTimeout(20000);
    window->set3DMode();
    window->addRenderer(regCloudRenderer);
    window->addRenderer(cloudRenderer);
    window->start();
}

}