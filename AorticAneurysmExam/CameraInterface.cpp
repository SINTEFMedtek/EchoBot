#include <FAST/Streamers/KinectStreamer.hpp>

#include "CameraInterface.hpp"

#include "FAST/Data/Image.hpp"
#include "FAST/Data/Mesh.hpp"
#include "FAST/Algorithms/IterativeClosestPoint/IterativeClosestPoint.hpp"
#include <FAST/Exporters/VTKMeshFileExporter.hpp>
#include <QDir>

namespace fast {

CameraInterface::CameraInterface() {

    createInputPort<Image>(0);
    createInputPort<Mesh>(1);

    createOutputPort<Image>(0);
    createOutputPort<Mesh>(1); // Annotation image

    getReporter().setReportMethod(Reporter::COUT);
}

void CameraInterface::restart() {
    stopRecording();
}

void CameraInterface::startRecording(std::string path) {
    mStoragePath = path;
    mFrameCounter = 0;
    mRecording = true;
}

void CameraInterface::stopRecording() {
    mRecording = false;
}

void CameraInterface::execute() {
    std::cout << "Enters execute " << mRecording << std::endl;

    Image::pointer input = getInputData<Image>(0);
    Mesh::pointer meshInput = getInputData<Mesh>(1);

    if(mRecording) {
        VTKMeshFileExporter::pointer exporter = VTKMeshFileExporter::New();
        exporter->setInputData(meshInput);
        exporter->setWriteNormals(false);
        exporter->setFilename(mStoragePath + std::to_string(mFrameCounter) + ".vtk");
        exporter->update(0);
        ++mFrameCounter;
    }

    addOutputData(0, input);
    addOutputData(1, meshInput);
    mCurrentCloud = meshInput;
}

uint CameraInterface::getFramesStored() const {
    return mFrameCounter;
}

bool CameraInterface::isRecording() const {
    return mRecording;
}

}