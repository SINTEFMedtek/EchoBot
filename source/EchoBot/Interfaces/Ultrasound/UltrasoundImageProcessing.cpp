//
// Created by androst on 19.08.19.
//

#include "UltrasoundImageProcessing.h"

#include "FAST/Algorithms/UltrasoundImageCropper/UltrasoundImageCropper.hpp"
#include "FAST/Algorithms/ImageCropper/ImageCropper.hpp"
#include "FAST/Algorithms/NeuralNetwork/SegmentationNetwork.hpp"
#include "FAST/Exporters/MetaImageExporter.hpp"


namespace echobot {

UltrasoundImageProcessing::UltrasoundImageProcessing() {
    createInputPort<Image>(0);

    createOutputPort<Image>(0);
    createOutputPort<Image>(1); // Segmentation

    mSegmentationThread = new std::thread(std::bind(&UltrasoundImageProcessing::segmentationThread, this));
    setupNeuralNetworks();
}

UltrasoundImageProcessing::~UltrasoundImageProcessing() {
    std::cout << "Stopping segmentation thread.." << std::endl;
    {
        std::lock_guard<std::mutex> lock(mFrameBufferMutex);
        mStop = true;
    }
    mSegmentationThread->join();
    std::cout << "Segmentation thread stopped!" << std::endl;
}

void UltrasoundImageProcessing::execute() {
    fast::Image::pointer input = getInputData<fast::Image>(0);

//    ImageCropper::pointer cropper = ImageCropper::New();
//    cropper->setInputData(input);
//    cropper->setOffset(Vector3i(50, 75, 0));
//    cropper->setSize(Vector3i(580, 470, 1));
//
//    auto cropPort = cropper->getOutputPort();
//    cropper->update(0, STREAMING_MODE_NEWEST_FRAME_ONLY);
//
//    mCurrentImage = cropPort->getNextFrame<fast::Image>();
//    mCurrentImage->setSpacing(Vector3f(0.435, 0.435, 1)); // Bug hack spacing (spacing*10/2
//
//    if (mRobotInterface->robot.isConnected())
//        transformImageToProbeCenter();
//
//    Image::pointer segmentation;
//    if (mSegmentationEnabled) {
//        mPixelClassifier->setInputData(mCurrentImage);
//        DataPort::pointer port = mPixelClassifier->getOutputPort(0);
//        mPixelClassifier->update(0, STREAMING_MODE_NEWEST_FRAME_ONLY);
//        segmentation = port->getNextFrame<Image>();
//        segmentation = mCurrentImage;
//    } else {
//        segmentation = mCurrentImage;
//    }
    auto segmentation = input;
    mCurrentImage = input;

    try {
        addOutputData(0, mCurrentImage);
        addOutputData(1, segmentation);
    } catch (ThreadStopped &e) {
        std::cout << "Thread stopped in USImageProcessing" << std::endl;
    }


}

void UltrasoundImageProcessing::segmentationThread() {
    while (true) {
        {
            std::lock_guard<std::mutex> lock(mFrameBufferMutex);
            if (mStop)
                break;
        }
    }
}

void UltrasoundImageProcessing::setupNeuralNetworks() {
//    mPixelClassifier = PixelClassifier::New();
//    mPixelClassifier->setNrOfClasses(2);
//    mPixelClassifier->setResizeBackToOriginalSize(false);
//    mPixelClassifier->load("/home/androst/Data/NNModels/phantom.pb");
//    mPixelClassifier->setScaleFactor(1.0f / 255.0f);
//    mPixelClassifier->addOutputNode(0, "conv2d_23/truediv");
}

}