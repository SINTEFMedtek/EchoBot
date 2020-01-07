#include "CameraDataProcessing.h"

#include "FAST/Data/Image.hpp"
#include "FAST/Data/Mesh.hpp"
#include "FAST/Algorithms/IterativeClosestPoint/IterativeClosestPoint.hpp"
#include <FAST/Exporters/VTKMeshFileExporter.hpp>
#include <FAST/Exporters/MetaImageExporter.hpp>
#include <QDir>
#include <random>

namespace echobot {

CameraDataProcessing::CameraDataProcessing() {
    createInputPort<Image>(0);
    createInputPort<Image>(1, false);
    createInputPort<Mesh>(2, false);

    createOutputPort<Image>(0); // RGB image clean
    createOutputPort<Image>(1); // RGB image annotated
    createOutputPort<Image>(2); // Depth image clean
    createOutputPort<Mesh>(3); // Point cloud clean
    createOutputPort<Mesh>(4); // Point cloud annotated

    // Create annotation image
    mAnnotationImage = Image::New();
    mAnnotationImage->create(512, 424, TYPE_UINT8, 1);
    mAnnotationImage->fill(0);

    mTargetCloud = Mesh::New();
    mTargetCloud->create(0, 0, 0, false, false, false);
    mTargetCloudExtracted = false;
}

void CameraDataProcessing::execute() {
    Image::pointer rgbInput = getInputData<Image>(0);
    mCurrentImage = rgbInput;

    if(mInputConnections.count(1)) {
        Image::pointer depthInput = getInputData<Image>(1);
        mCurrentDepthImage = depthInput;
    }

    if(mInputConnections.count(2)){
        Mesh::pointer meshInput = getInputData<Mesh>(2);
        mCurrentCloud = meshInput;

        auto cloudAccess = mCurrentCloud->getMeshAccess(ACCESS_READ);
        std::vector<MeshVertex> vertices = cloudAccess->getVertices();
        std::vector<MeshVertex> filteredPoints;
        for(auto vertex: vertices) {
            auto position = vertex.getPosition();
            if (!position.z() || position.z() > mMaxRange || position.z() < mMinRange)
                continue;

            if(position.x() > mMaxWidth || position.x() < mMinWidth)
                continue;

            if(position.y() > mMaxHeight || position.y() < mMinHeight)
                continue;

            filteredPoints.push_back(vertex);
        }

        Mesh::pointer filteredCloud = Mesh::New();
        filteredCloud->create(filteredPoints);
        filteredCloud->setCreationTimestamp(mCurrentCloud->getCreationTimestamp());
        mCurrentCloud = filteredCloud;
    }

    if (mTargetCloudExtracted) {
        //reportInfo() << "Running ICP" << reportEnd();
        IterativeClosestPoint::pointer icp = IterativeClosestPoint::New();
        //icp->enableRuntimeMeasurements();
        icp->setFixedMesh(mCurrentCloud);
        icp->setMovingMesh(mTargetCloud);
        icp->setDistanceThreshold(100); // All points further away than 10 cm from the centroid is removed
        //icp->setMinimumErrorChange(0.5);
        icp->setRandomPointSampling(300);
        //icp->getReporter().setReportMethod(Reporter::COUT);
        icp->setMaximumNrOfIterations(20);
        icp->update();
        //reportInfo() << "Finished ICP in: " << reportEnd();
        //icp->getAllRuntimes()->printAll();
        if (!mTargetCloudPlaced) {
            AffineTransformation::pointer currentTransform = mTargetCloud->getSceneGraphNode()->getTransformation();
            AffineTransformation::pointer newTransform = icp->getOutputTransformation();
            mTargetCloud->getSceneGraphNode()->setTransformation(newTransform->multiply(currentTransform));
            mTargetCloudPlaced = true;
        }
    } else {
        mTargetCloud = mCurrentCloud;
    }

    addOutputData(0, mCurrentImage);
    addOutputData(1, mAnnotationImage);
    addOutputData(2, mCurrentDepthImage);
    addOutputData(3, mCurrentCloud);
    addOutputData(4, mTargetCloud);
}

void CameraDataProcessing::addLine(Vector2i start, Vector2i end) {
    std::cout << "Drawing from: " << start.transpose() << " to " << end.transpose() << std::endl;
    // Draw line in some auxillary image
    mAnnotationImage = mAnnotationImage->copy(Host::getInstance());
    ImageAccess::pointer access = mAnnotationImage->getImageAccess(ACCESS_READ_WRITE);
    Vector2f direction = end.cast<float>() - start.cast<float>();
    int length = (end - start).norm();
    int brushSize = 6;
    for (int i = 0; i < length; ++i) {
        float distance = (float) i / length;
        for (int a = -brushSize; a <= brushSize; a++) {
            for (int b = -brushSize; b <= brushSize; b++) {
                Vector2f offset(a, b);
                if (offset.norm() > brushSize)
                    continue;
                Vector2f position = start.cast<float>() + direction * distance + offset;
                try {
                    access->setScalar(position.cast<int>(), 1);
                } catch (Exception &e) {

                }
            }
        }
    }
}

void CameraDataProcessing::calculateTargetCloud(RealSenseStreamer::pointer streamer) {
    std::cout << "Creating target cloud..." << std::endl;
    ImageAccess::pointer access = mAnnotationImage->getImageAccess(ACCESS_READ);
    MeshAccess::pointer meshAccess = mCurrentCloud->getMeshAccess(ACCESS_READ);
    std::vector<MeshVertex> vertices = meshAccess->getVertices();
    std::vector<MeshVertex> outputVertices;
    for (int y = 0; y < mAnnotationImage->getHeight(); ++y) {
        for (int x = 0; x < mAnnotationImage->getWidth(); ++x) {
            try {
                if (access->getScalar(Vector2i(x, y)) == 1) {
                    MeshVertex vertex = streamer->getPoint(x, y);
                    if (!std::isnan(vertex.getPosition().x())) {
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

SharedPointer<Mesh> CameraDataProcessing::getTargetCloud() {
    return mTargetCloud;
}

void CameraDataProcessing::removeTargetCloud() {
    mTargetCloudExtracted = false;
    mAnnotationImage->fill(0);
}

void CameraDataProcessing::setMaxRange(float range) {
    if(range < 0)
        throw Exception("Range has to be >= 0");
    mMaxRange = range;
}

void CameraDataProcessing::setMinRange(float range) {
    if(range < 0)
        throw Exception("Range has to be >= 0");
    mMinRange = range;
}

void CameraDataProcessing::setMaxWidth(float range) {
    mMaxWidth = range;
}

void CameraDataProcessing::setMinWidth(float range) {
    mMinWidth = range;
}

void CameraDataProcessing::setMaxHeight(float range) {
    mMaxHeight = range;
}

void CameraDataProcessing::setMinHeight(float range) {
    mMinHeight = range;
}

}