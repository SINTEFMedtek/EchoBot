#ifndef ECHOBOT_ULTRASOUNDINTERFACE_H
#define ECHOBOT_ULTRASOUNDINTERFACE_H

#include <thread>
#include "EchoBot/Core/SmartPointers.h"
#include "EchoBot/Interfaces/Robot/RobotInterface.h"
#include "EchoBot/Interfaces/Ultrasound/UltrasoundImageProcessing.h"

#include "FAST/ProcessObject.hpp"
#include "FAST/Data/Image.hpp"
#include "FAST/Streamers/ClariusStreamer.hpp"
#include "FAST/Streamers/OpenIGTLinkStreamer.hpp"
#include "FAST/Visualization/Renderer.hpp"

namespace echobot
{
using namespace fast;

class UltrasoundInterface : public SensorInterface {
    ECHOBOT_OBJECT(UltrasoundInterface)

    public:
        typedef enum {Clarius, IGTLink} UltrasoundStreamerType; // Supported ultrasound streamers

        ~UltrasoundInterface();
        void connect();
        void disconnect();
        bool isConnected(){return mConnected;};

        void setStreamer(UltrasoundStreamerType streamer, std::string ip = "", uint32_t port = 18944);

        DataChannel::pointer getOutputPort(uint portID = 0);

        Streamer::pointer getStreamObject(){ return mUltrasoundStreamer;};
        UltrasoundImageProcessing::pointer getProcessObject(){ return mProcessObject;};
        Renderer::pointer getRendererObject();

private:
        UltrasoundInterface();
        UltrasoundStreamerType mStreamerType;
        std::string mIP;
        uint32_t mPort;

        SharedPointer<Streamer> mUltrasoundStreamer;
        SharedPointer<UltrasoundImageProcessing> mProcessObject;
        SharedPointer<Renderer> mRendererObject;

        SharedPointer<RobotInterface> mRobotInterface;

        bool mConnected = false;
};



} // end namespace echobot
#endif
