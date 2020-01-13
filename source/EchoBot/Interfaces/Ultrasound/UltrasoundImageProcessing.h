//
// Created by androst on 19.08.19.
//

#ifndef ECHOBOT_ULTRASOUNDIMAGEPROCESSING_H
#define ECHOBOT_ULTRASOUNDIMAGEPROCESSING_H

#include <thread>
#include "EchoBot/Core/SmartPointers.h"
#include "EchoBot/Core/DataTypes.h"

#include "FAST/ProcessObject.hpp"
#include "FAST/Data/Image.hpp"

namespace echobot {


class PixelClassifier;

class UltrasoundImageProcessing : public ProcessObject {
    ECHOBOT_OBJECT(UltrasoundImageProcessing)

    public:
        ~UltrasoundImageProcessing();

        SharedPointer<fast::Image> getProcessedImage(){ return mProcessedImage;};
        void setImageTransform(Eigen::Affine3d transform);

    private:
        UltrasoundImageProcessing();

        void execute();

        SharedPointer<fast::Image> mRawImage, mProcessedImage;
        Eigen::Affine3d mImageTransform;

        void segmentationThread();

        std::thread *mSegmentationThread;
        std::mutex mFrameBufferMutex;

        bool mSegmentationEnabled = true;
        SharedPointer<PixelClassifier> mPixelClassifier;

        void setupNeuralNetworks();

        bool mStop = false;
    };
}


#endif //ECHOBOT_ULTRASOUNDIMAGEPROCESSING_H
