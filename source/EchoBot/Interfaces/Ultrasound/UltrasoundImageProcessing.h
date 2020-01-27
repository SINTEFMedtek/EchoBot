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


class UltrasoundImageProcessing : public ProcessObject {
    ECHOBOT_OBJECT(UltrasoundImageProcessing)

    public:
        ~UltrasoundImageProcessing();

        SharedPointer<fast::Image> getProcessedImage(){ return mProcessedImage;};
        void setImageTransform(Eigen::Affine3d transform);
        bool isSegmentationEnabled(){ return mSegmentationEnabled;};
        void setupNeuralNetworks();

    private:
        UltrasoundImageProcessing();

        void execute();

        SharedPointer<fast::Image> mRawImage, mProcessedImage;
        Eigen::Affine3d mImageTransform;

        void segmentationThread();

        std::thread *mSegmentationThread;
        std::mutex mFrameBufferMutex;

        bool mSegmentationEnabled = false;
        SharedPointer<SegmentationNetwork> mSegmentationNetwork;


        bool mStop = false;
    };
}


#endif //ECHOBOT_ULTRASOUNDIMAGEPROCESSING_H
