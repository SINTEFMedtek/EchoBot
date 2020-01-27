#include "UltrasoundInterface.hpp"

#include "FAST/Streamers/OpenIGTLinkStreamer.hpp"
#include "FAST/Streamers/ClariusStreamer.hpp"
#include "FAST/Streamers/ImageFileStreamer.hpp"
#include "FAST/Visualization/ImageRenderer/ImageRenderer.hpp"

namespace echobot
{

UltrasoundInterface::UltrasoundInterface() {
    fast::Config::setStreamingMode(fast::STREAMING_MODE_NEWEST_FRAME_ONLY);
}

UltrasoundInterface::~UltrasoundInterface() {
}

void UltrasoundInterface::connect()
{
    if(mStreamerType == Clarius){
        mUltrasoundStreamer = fast::ClariusStreamer::New();
    }
    else if(mStreamerType == IGTLink){
        auto usStreamer = fast::OpenIGTLinkStreamer::New();
        usStreamer->setConnectionAddress(mIP);
        usStreamer->setConnectionPort(mPort);
        mUltrasoundStreamer = usStreamer;
    }
    else if(mStreamerType == Playback){
        auto usStreamer = fast::ImageFileStreamer::New();
        usStreamer->setFilenameFormat(mPlaybackFilepath);
        usStreamer->enableLooping();
        usStreamer->setSleepTime(33.3);
        mUltrasoundStreamer = usStreamer;
    }

    mProcessObject = UltrasoundImageProcessing::New();
    mProcessObject->setInputConnection(mUltrasoundStreamer->getOutputPort());
    mConnected = true;
}

void UltrasoundInterface::disconnect()
{
    mUltrasoundStreamer->stopPipeline();
    mImageRenderer->stopPipeline();
    mSegmentationRenderer->stopPipeline();
    mProcessObject->stopPipeline();
    mConnected = false;
}

void UltrasoundInterface::setStreamer(UltrasoundStreamerType streamer, std::string ip, uint32_t port)
{
    mStreamerType = streamer;
    mIP = ip;
    mPort = port;
}

UltrasoundInterface::UltrasoundStreamerType UltrasoundInterface::getStreamerType()
{
    return mStreamerType;
}

void UltrasoundInterface::setPlayback(std::string filepath)
{
    mStreamerType = UltrasoundStreamerType::Playback;
    mPlaybackFilepath = filepath;
}

DataChannel::pointer UltrasoundInterface::getOutputPort(uint portID)
{
    return mProcessObject->getOutputPort(portID);
}

Renderer::pointer UltrasoundInterface::getImageRenderer()
{
    mImageRenderer = fast::ImageRenderer::New();
    mImageRenderer->addInputConnection(mProcessObject->getOutputPort(0));
    return mImageRenderer;
}


void UltrasoundInterface::setImageTransform(Eigen::Affine3d transform) {
    mProcessObject->setImageTransform(transform);
}

Renderer::pointer UltrasoundInterface::getSegmentationRenderer() {
    auto segmentationRenderer = fast::SegmentationRenderer::New();
    segmentationRenderer->addInputConnection(mProcessObject->getOutputPort(1));
    segmentationRenderer->setOpacity(0.25);
    segmentationRenderer->setColor(fast::Segmentation::LABEL_FOREGROUND, Color::Red());
    segmentationRenderer->setColor(fast::Segmentation::LABEL_BLOOD, Color::Black());
    mSegmentationRenderer = segmentationRenderer;
    return mSegmentationRenderer;
}

void UltrasoundInterface::enableSegmentation() {
    mProcessObject->setupNeuralNetworks();
}

}