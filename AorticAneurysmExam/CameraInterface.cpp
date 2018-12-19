#include "CameraInterface.hpp"

#include "FAST/Data/Image.hpp"
#include "FAST/Data/Mesh.hpp"
#include "FAST/Algorithms/IterativeClosestPoint/IterativeClosestPoint.hpp"
#include <FAST/Exporters/VTKMeshFileExporter.hpp>
#include <FAST/Exporters/MetaImageExporter.hpp>
#include <FAST/Streamers/KinectStreamer.hpp>
#include <QDir>

namespace fast {

CameraInterface::CameraInterface() {

    createInputPort<Image>(0);
    createInputPort<Mesh>(1);

    createOutputPort<Image>(0);
    createOutputPort<Image>(1); // Annotation image
    createOutputPort<Mesh>(2);
    createOutputPort<Mesh>(3); // Annotated cloud

    // Create annotation image
    mAnnotationImage = Image::New();
    mAnnotationImage->create(512, 424, TYPE_UINT8, 1);
    mAnnotationImage->fill(0);

    mTargetCloud = Mesh::New();
    mTargetCloud->create(0, 0, 0, false, false, false);
    mTargetCloudExtracted = false;

}

void CameraInterface::restart() {
    stopRecording();
}

void CameraInterface::startRecording(std::string path) {
    mStoragePath = path;
    mFrameCounter = 0;
    mRecording = true;

    createDirectories((mStoragePath + "/PointClouds"));
    createDirectories((mStoragePath + "/CameraImages"));
}

void CameraInterface::stopRecording() {
    mRecording = false;
}

void CameraInterface::execute() {
    Image::pointer input = getInputData<Image>(0);
    Mesh::pointer meshInput = getInputData<Mesh>(1);

    mCurrentImage = input;
    mCurrentCloud = meshInput;

    if(mTargetCloudExtracted)
    {
        //reportInfo() << "Running ICP" << reportEnd();
        IterativeClosestPoint::pointer icp = IterativeClosestPoint::New();
        //icp->enableRuntimeMeasurements();
        icp->setFixedMesh(meshInput);
        icp->setMovingMesh(mTargetCloud);
        icp->setDistanceThreshold(100); // All points further away than 10 cm from the centroid is removed
        //icp->setMinimumErrorChange(0.5);
        icp->setRandomPointSampling(300);
        //icp->getReporter().setReportMethod(Reporter::COUT);
        icp->setMaximumNrOfIterations(20);
        icp->update(0);
        //reportInfo() << "Finished ICP in: " << reportEnd();
        //icp->getAllRuntimes()->printAll();
        if(!mTargetCloudPlaced){
            AffineTransformation::pointer currentTransform = mTargetCloud->getSceneGraphNode()->getTransformation();
            AffineTransformation::pointer newTransform = icp->getOutputTransformation();
            mTargetCloud->getSceneGraphNode()->setTransformation(newTransform->multiply(currentTransform));
            mTargetCloudPlaced = true;
        }
    } else{
        mTargetCloud = meshInput;
    }

    if(mRecording) {
        VTKMeshFileExporter::pointer meshExporter = VTKMeshFileExporter::New();
        meshExporter->setInputData(meshInput);
        meshExporter->setWriteNormals(false);
        meshExporter->setWriteColors(true);
        meshExporter->setFilename(mStoragePath + "/PointClouds/" + std::to_string(mFrameCounter) + ".vtk");
        meshExporter->update(0);

        MetaImageExporter::pointer imageExporter = MetaImageExporter::New();
        imageExporter->setInputData(input);
        imageExporter->setFilename(mStoragePath + "/CameraImages/" + "Cam-2D_" + std::to_string(mFrameCounter) + ".mhd");
        imageExporter->update(0);

        ++mFrameCounter;
    }

    addOutputData(0, mCurrentImage);
    addOutputData(1, mAnnotationImage);
    addOutputData(2, mCurrentCloud);
    addOutputData(3, mTargetCloud);
}

uint CameraInterface::getFramesStored() const {
    return mFrameCounter;
}

bool CameraInterface::isRecording() const {
    return mRecording;
}

void CameraInterface::addLine(Vector2i start, Vector2i end) {
    std::cout << "Drawing from: " << start.transpose() << " to " << end.transpose() << std::endl;
    // Draw line in some auxillary image
    mAnnotationImage = mAnnotationImage->copy(Host::getInstance());
    ImageAccess::pointer access = mAnnotationImage->getImageAccess(ACCESS_READ_WRITE);
    Vector2f direction = end.cast<float>() - start.cast<float>();
    int length = (end-start).norm();
    int brushSize = 6;
    for(int i = 0; i < length; ++i) {
        float distance = (float)i/length;
        for(int a = -brushSize; a <= brushSize; a++) {
            for(int b = -brushSize; b <= brushSize; b++) {
                Vector2f offset(a, b);
                if(offset.norm() > brushSize)
                    continue;
                Vector2f position = start.cast<float>() + direction*distance + offset;
                try {
                    access->setScalar(position.cast<int>(), 1);
                } catch(Exception &e) {

                }
            }
        }
    }
}

void CameraInterface::calculateTargetCloud(KinectStreamer::pointer streamer) {
    std::cout << "Creating target cloud..." << std::endl;
    ImageAccess::pointer access = mAnnotationImage->getImageAccess(ACCESS_READ);
    MeshAccess::pointer meshAccess = mCurrentCloud->getMeshAccess(ACCESS_READ);
    std::vector<MeshVertex> vertices = meshAccess->getVertices();
    std::vector<MeshVertex> outputVertices;
    for(int y = 0; y < mAnnotationImage->getHeight(); ++y) {
        for(int x = 0; x < mAnnotationImage->getWidth(); ++x) {
            try {
                if(access->getScalar(Vector2i(x, y)) == 1) {
                    MeshVertex vertex = streamer->getPoint(x, y);
                    if(!std::isnan(vertex.getPosition().x())) {
                        outputVertices.push_back(vertex);
                    }
                }
            } catch (Exception &e) {

            }
        }
    }

    mTargetCloud = Mesh::New();
    mTargetCloud->create(outputVertices);
    std::cout << "Created target cloud." << std::endl;
    mTargetCloudExtracted = true;
}

SharedPointer<Mesh> CameraInterface::getTargetCloud(){
    return mTargetCloud;
}

void CameraInterface::removeTargetCloud()
{
    mTargetCloudExtracted = false;
    mAnnotationImage->fill(0);
}

Mesh::pointer CameraInterface::createReducedSample(Mesh::pointer pointCloud, double fractionOfPointsToKeep) {
    MeshAccess::pointer accessFixedSet = pointCloud->getMeshAccess(ACCESS_READ);
    std::vector<MeshVertex> vertices = accessFixedSet->getVertices();

    // Sample the preferred amount of points from the point cloud
    auto numVertices = (unsigned int) vertices.size();
    auto numSamplePoints = (unsigned int) ceil(fractionOfPointsToKeep * numVertices);
    std::vector<MeshVertex> newVertices;

    std::unordered_set<int> movingIndices;
    unsigned int sampledPoints = 0;
    std::default_random_engine distributionEngine;
    std::uniform_int_distribution<unsigned int> distribution(0, numVertices - 1);
    while (sampledPoints < numSamplePoints) {
        unsigned int index = distribution(distributionEngine);
        if (movingIndices.count(index) < 1 && vertices.at(index).getPosition().array().isNaN().sum() == 0) {
            newVertices.push_back(vertices.at(index));
            movingIndices.insert(index);
            ++sampledPoints;
        }
    }

    // Add noise to point cloud
    float minX, minY, minZ;
    Vector3f position0 = vertices[0].getPosition();
    minX = position0[0];
    minY = position0[1];
    minZ = position0[2];
    float maxX = minX, maxY = minY, maxZ = minZ;
    for (auto &vertex : vertices) {
        Vector3f position = vertex.getPosition();
        if (position[0] < minX) { minX = position[0]; }
        if (position[0] > maxX) { maxX = position[0]; }
        if (position[1] < minY) { minY = position[1]; }
        if (position[1] > maxY) { maxY = position[1]; }
        if (position[2] < minZ) { minZ = position[2]; }
        if (position[2] > maxZ) { maxZ = position[2]; }
    }
    Mesh::pointer newCloud = Mesh::New();
    newCloud->create(newVertices);
    // Update point cloud to include the removed points and added noise
    return newCloud;
}




}