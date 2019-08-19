#include "UltrasoundInterface.hpp"

#include "FAST/Streamers/OpenIGTLinkStreamer.hpp"
#include "FAST/Streamers/ClariusStreamer.hpp"
#include "FAST/Visualization/ImageRenderer/ImageRenderer.hpp"

namespace echobot
{

UltrasoundInterface::UltrasoundInterface() {
}

UltrasoundInterface::~UltrasoundInterface() {
}

void UltrasoundInterface::connect()
{
    if(mStreamerType == Clarius)
        mUltrasoundStreamer = ClariusStreamer::New();
    else if(mStreamerType == IGTLink){
        auto usStreamer = OpenIGTLinkStreamer::New();
        usStreamer->setConnectionAddress(mIP);
        usStreamer->setConnectionPort(mPort);
        mUltrasoundStreamer = usStreamer;
    }

    mProcessObject = UltrasoundImageProcessing::New();
    mProcessObject->setInputConnection(mUltrasoundStreamer->getOutputPort());
}

void UltrasoundInterface::disconnect()
{
    mRendererObject->stopPipeline();
    mProcessObject->stopPipeline();
    mUltrasoundStreamer->stopPipeline();
}

void UltrasoundInterface::setStreamer(UltrasoundStreamerType streamer, std::string ip, uint32_t port)
{
    mStreamerType = streamer;
    mIP = ip;
    mPort = port;
}

DataChannel::pointer UltrasoundInterface::getOutputPort(uint portID)
{
    return mProcessObject->getOutputPort(portID);
}

Renderer::pointer UltrasoundInterface::getRendererObject()
{
    mRendererObject = ImageRenderer::New();
    mRendererObject->addInputConnection(mProcessObject->getOutputPort());
    return mRendererObject;
}

}