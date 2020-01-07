#include "CameraInterface.hpp"
#include "FAST/Data/Image.hpp"
#include "FAST/Data/Mesh.hpp"
#include "FAST/Algorithms/IterativeClosestPoint/IterativeClosestPoint.hpp"
#include <FAST/Exporters/VTKMeshFileExporter.hpp>
#include <FAST/Streamers/RealSenseStreamer.hpp>
#include <FAST/Exporters/MetaImageExporter.hpp>
#include <FAST/Streamers/ImageFileStreamer.hpp>
#include <FAST/Streamers/MeshFileStreamer.hpp>
#include <FAST/Visualization/VertexRenderer/VertexRenderer.hpp>
#include <FAST/Visualization/ImageRenderer/ImageRenderer.hpp>
#include <QDir>


namespace echobot {

DataChannel::pointer CameraInterface::getOutputPort(uint portID)
{
    return mProcessObject->getOutputPort(portID);
}

void CameraInterface::setCameraROI(float minRange, float maxRange, float minWidth, float maxWidth, float minHeight, float maxHeight)
{
    mProcessObject->setMinRange(minRange);
    mProcessObject->setMaxRange(maxRange);
    mProcessObject->setMinWidth(minWidth);
    mProcessObject->setMaxWidth(maxWidth);
    mProcessObject->setMinHeight(minHeight);
    mProcessObject->setMaxHeight(maxHeight);
}

void CameraInterface::connect()
{
    mProcessObject = CameraDataProcessing::New();

    if(mStreamOption == StreamOption::Stream){
        mCameraStreamer = RealSenseStreamer::New();
        mProcessObject->setInputConnection(0, mCameraStreamer->getOutputPort(0));
        mProcessObject->setInputConnection(1, mCameraStreamer->getOutputPort(1));
        mProcessObject->setInputConnection(2, mCameraStreamer->getOutputPort(2));
    } else if (mStreamOption == StreamOption::Playback) {
        auto imageStreamer = ImageFileStreamer::New();
        imageStreamer->setFilenameFormat(mPlaybackFilepath + "/CameraImages/Image-2D_#.mhd");
        imageStreamer->enableLooping();
        imageStreamer->setSleepTime(33.3);
        mCameraStreamer = imageStreamer;

        auto meshStreamer = MeshFileStreamer::New();
        meshStreamer->setFilenameFormat(mPlaybackFilepath + "/PointClouds/#.vtk");
        meshStreamer->enableLooping();
        meshStreamer->setSleepTime(33.3);
        meshStreamer->update();

        mProcessObject->setInputConnection(0, mCameraStreamer->getOutputPort(0));
        mProcessObject->setInputConnection(2, meshStreamer->getOutputPort(0));
    }
    mConnected = true;
}

void CameraInterface::disconnect()
{
    mProcessObject->stopPipeline();
    mCameraStreamer->stopPipeline();
    mImageRenderer->stopPipeline();
    mDepthImageRenderer->stopPipeline();
    mPointCloudRenderer->stopPipeline();
    mConnected = false;
}

Renderer::pointer CameraInterface::getImageRenderer()
{
    // Renderer RGB image
    mImageRenderer = ImageRenderer::New();
    mImageRenderer->addInputConnection(mProcessObject->getOutputPort(0));
    return mImageRenderer;
}

Renderer::pointer CameraInterface::getDepthImageRenderer()
{
    // Renderer depth image
    mDepthImageRenderer = ImageRenderer::New();
    mDepthImageRenderer->addInputConnection(mProcessObject->getOutputPort(2));
    mDepthImageRenderer->setIntensityLevel(1000);
    mDepthImageRenderer->setIntensityWindow(500);
    return mDepthImageRenderer;
}

Renderer::pointer CameraInterface::getPointCloudRenderer()
{
    // Renderer point cloud
    mPointCloudRenderer = VertexRenderer::New();
    mPointCloudRenderer->addInputConnection(mProcessObject->getOutputPort(4));
    mPointCloudRenderer->setDefaultSize(1.5);
    return mPointCloudRenderer;
}

void CameraInterface::setPlayback(std::string filepath)
{
    mPlaybackFilepath = filepath;
    mStreamOption = StreamOption::Playback;
}


CameraInterface::CameraInterface() {
    mPointCloudRenderer = VertexRenderer::New();
    mDepthImageRenderer = ImageRenderer::New();
    mImageRenderer = ImageRenderer::New();
}

CameraInterface::~CameraInterface() {
}

}