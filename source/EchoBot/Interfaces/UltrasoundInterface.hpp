#ifndef FASTROMO_ULTRASOUNDINTERFACE_H
#define FASTROMO_ULTRASOUNDINTERFACE_H

#include "FAST/ProcessObject.hpp"
#include "RobotInterface.h"
#include <thread>

namespace fast {

class Image;
class PixelClassifier;

class UltrasoundInterface : public ProcessObject {
    FAST_OBJECT(UltrasoundInterface)

    public:
        void setRobotInterface(RobotInterfacePtr robotInterface);
        ~UltrasoundInterface();

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

        static SharedPointer<PixelClassifier> mPixelClassifier;
        void setupNeuralNetworks();

        bool mStop = false;
};

}

#endif
