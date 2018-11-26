#include <FAST/Streamers/KinectStreamer.hpp>

#include "CameraInterface.hpp"

#include "FAST/Data/Image.hpp"
#include "FAST/Data/Mesh.hpp"
#include "FAST/Algorithms/IterativeClosestPoint/IterativeClosestPoint.hpp"
#include <FAST/Exporters/VTKMeshFileExporter.hpp>
#include <FAST/Exporters/MetaImageExporter.hpp>
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

    createDirectories((mStoragePath + "/PointClouds"));
    createDirectories((mStoragePath + "/CameraImages"));
}

void CameraInterface::stopRecording() {
    mRecording = false;
}

void CameraInterface::execute() {
    Image::pointer input = getInputData<Image>(0);
    Mesh::pointer meshInput = getInputData<Mesh>(1);

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