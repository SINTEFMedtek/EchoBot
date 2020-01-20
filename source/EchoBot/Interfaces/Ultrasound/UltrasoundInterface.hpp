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

class UltrasoundInterface : public SensorInterface {
    ECHOBOT_OBJECT(UltrasoundInterface)

    public:
        typedef enum {Clarius, IGTLink, Playback} UltrasoundStreamerType; // Supported ultrasound streamers

        ~UltrasoundInterface();
        void connect();
        void disconnect();
        bool isConnected(){return mConnected;};

        void setStreamer(UltrasoundStreamerType streamer, std::string ip = "", uint32_t port = 18944);
        void setPlayback(std::string filepath);
        void setImageTransform(Eigen::Affine3d transform);

        DataChannel::pointer getOutputPort(uint portID = 0);

        Streamer::pointer getStreamObject(){ return mUltrasoundStreamer;};
        UltrasoundImageProcessing::pointer getProcessObject(){ return mProcessObject;};
        Renderer::pointer getRendererObject();
        UltrasoundStreamerType getStreamerType();

    private:
        UltrasoundInterface();
        UltrasoundStreamerType mStreamerType;
        std::string mIP = "localhost";
        uint32_t mPort = 18944;
        std::string mPlaybackFilepath = "";

        SharedPointer<Streamer> mUltrasoundStreamer;
        SharedPointer<UltrasoundImageProcessing> mProcessObject;
        SharedPointer<Renderer> mRendererObject;

        bool mConnected = false;
};

} // end namespace echobot
#endif
