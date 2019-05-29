#ifndef ECHOBOT_ULTRASOUNDINTERFACE_H
#define ECHOBOT_ULTRASOUNDINTERFACE_H

#include "FAST/ProcessObject.hpp"
#include "FAST/Data/Image.hpp"

#include "RobotInterface.h"
#include <thread>

using namespace fast;

class PixelClassifier;

class UltrasoundInterface : public ProcessObject {
    FAST_OBJECT(UltrasoundInterface)

    public:
        void setRobotInterface(RobotInterfacePtr robotInterface);
        ~UltrasoundInterface();

        void startRecording(std::string path);

    private:
        UltrasoundInterface();


        void execute();

        SharedPointer<Image> mCurrentImage;

        RobotInterfacePtr mRobotInterface;
        void transformImageToProbeCenter();

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

#endif
