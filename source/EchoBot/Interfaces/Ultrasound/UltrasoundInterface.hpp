#ifndef ECHOBOT_ULTRASOUNDINTERFACE_H
#define ECHOBOT_ULTRASOUNDINTERFACE_H

#include <thread>
#include "EchoBot/Core/SmartPointers.h"
#include "EchoBot/Interfaces/Robot/RobotInterface.h"

#include "FAST/ProcessObject.hpp"
#include "FAST/Data/Image.hpp"
#include "FAST/Streamers/ClariusStreamer.hpp"
#include "FAST/Streamers/OpenIGTLinkStreamer.hpp"
#include "FAST/Visualization/Renderer.hpp"

namespace echobot
{
using namespace fast;


class PixelClassifier;

class UltrasoundImageProcessing : public ProcessObject
{
    ECHOBOT_OBJECT(UltrasoundImageProcessing)

    public:
        ~UltrasoundImageProcessing();

        void startRecording(std::string path);
        void stopRecording();

    private:
        UltrasoundImageProcessing();

        void execute();

        SharedPointer<fast::Image> mCurrentImage;

        void segmentationThread();
        std::thread* mSegmentationThread;
        std::mutex mFrameBufferMutex;
        bool mSegmentationEnabled = true;

        SharedPointer<PixelClassifier> mPixelClassifier;
        void setupNeuralNetworks();

        bool mStop = false;
        bool mRecording = false;
        std::string mStoragePath;
        uint mFrameCounter;
};


class UltrasoundInterface : public SensorInterface {
    ECHOBOT_OBJECT(UltrasoundInterface)

    public:
        typedef enum {Clarius, IGTLink} UltrasoundStreamerType; // Supported ultrasound streamers

        ~UltrasoundInterface();
        void connect();
        void disconnect();
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
};



} // end namespace echobot
#endif
