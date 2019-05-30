#ifndef ECHOBOT_ULTRASOUNDINTERFACE_H
#define ECHOBOT_ULTRASOUNDINTERFACE_H

#include <thread>
#include "EchoBot/Core/SmartPointers.h"
#include "RobotInterface.h"

#include "FAST/ProcessObject.hpp"
#include "FAST/Data/Image.hpp"
#include "FAST/Streamers/Streamer.hpp"

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
        ~UltrasoundInterface();
        void connect();
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
