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
    mRendererObject->stopPipeline();
    mProcessObject->stopPipeline();
    mConnected = false;
}

void UltrasoundInterface::setStreamer(UltrasoundStreamerType streamer, std::string ip, uint32_t port)
{
    mStreamerType = streamer;
    mIP = ip;
    mPort = port;
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

Renderer::pointer UltrasoundInterface::getRendererObject()
{
    mRendererObject = fast::ImageRenderer::New();
    mRendererObject->addInputConnection(mProcessObject->getOutputPort(1));
    return mRendererObject;
}

void UltrasoundInterface::setImageTransform(Eigen::Affine3d transform) {
    mProcessObject->setImageTransform(transform);
}

}