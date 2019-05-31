#ifndef ECHOBOT_ULTRASOUNDINTERFACE_H
#define ECHOBOT_ULTRASOUNDINTERFACE_H

#include <thread>
#include "EchoBot/Core/SmartPointers.h"
#include "RobotInterface.h"

#include "FAST/ProcessObject.hpp"
#include "FAST/Data/Image.hpp"
#include "FAST/Streamers/ClariusStreamer.hpp"
#include "FAST/Streamers/IGTLinkStreamer.hpp"

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
        typedef enum {Clarius, IGTLink} UltrasoundStreamer; // Supported ultrasound streamers

        ~UltrasoundInterface();
        void connect();
        void setStreamer(UltrasoundStreamer streamer, std::string ip = "", uint32_t port = 18944);

        UltrasoundImageProcessing::pointer getProcessObject(){ return mProcessObject;};
        DataPort::pointer getOutputPort(uint portID = 0);


    private:
        UltrasoundInterface();

        SharedPointer<Streamer> mUltrasoundStreamer;
        SharedPointer<UltrasoundImageProcessing> mProcessObject;
        SharedPointer<RobotInterface> mRobotInterface;
};



} // end namespace echobot
#endif
