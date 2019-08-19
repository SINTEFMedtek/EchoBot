#include "CameraInterface.hpp"

#include "FAST/Data/Image.hpp"
#include "FAST/Data/Mesh.hpp"
#include "FAST/Algorithms/IterativeClosestPoint/IterativeClosestPoint.hpp"
#include <FAST/Exporters/VTKMeshFileExporter.hpp>
#include <FAST/Exporters/MetaImageExporter.hpp>
#include <FAST/Streamers/RealSenseStreamer.hpp>
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
    mCameraStreamer->setMinRange(minRange);
    mCameraStreamer->setMaxRange(maxRange);
    mCameraStreamer->setMinWidth(minWidth);
    mCameraStreamer->setMaxWidth(maxWidth);
    mCameraStreamer->setMinHeight(minHeight);
    mCameraStreamer->setMaxHeight(maxHeight);
}

void CameraInterface::connect()
{
    mCameraStreamer = RealSenseStreamer::New();
    mProcessObject = CameraDataProcessing::New();
    mProcessObject->setInputConnection(0, mCameraStreamer->getOutputPort(0));
    mProcessObject->setInputConnection(1, mCameraStreamer->getOutputPort(1));
    mProcessObject->setInputConnection(2, mCameraStreamer->getOutputPort(2));
}

void CameraInterface::disconnect()
{
    mProcessObject->stopPipeline();
    mCameraStreamer->stopPipeline();
    mImageRenderer->stopPipeline();
    mDepthImageRenderer->stopPipeline();
    //mPointCloudRenderer->stopPipeline();
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

CameraInterface::CameraInterface() {
}

CameraInterface::~CameraInterface() {
}

}