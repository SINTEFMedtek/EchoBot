#ifndef ECHOBOT_ULTRASOUNDINTERFACE_H
#define ECHOBOT_ULTRASOUNDINTERFACE_H

#include <thread>
#include "EchoBot/Core/SmartPointers.h"
#include "RobotInterface.h"

#include "FAST/ProcessObject.hpp"
#include "FAST/Data/Image.hpp"

namespace echobot
{
    using namespace fast;

    class PixelClassifier;

    class UltrasoundInterface : public ProcessObject {
        ECHOBOT_OBJECT(UltrasoundInterface)

        public:
            void setRobotInterface(RobotInterfacePtr robotInterface);
            ~UltrasoundInterface();
            void startRecording(std::string path);

        private:
            UltrasoundInterface();


            void execute();

            SharedPointer<fast::Image> mCurrentImage;

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

} // end namespace echobot
#endif
