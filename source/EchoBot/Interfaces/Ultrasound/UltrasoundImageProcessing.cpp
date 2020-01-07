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

    createOutputPort<Image>(0); // Raw image
    createOutputPort<Image>(1); // Processed image

    mSegmentationThread = new std::thread(std::bind(&UltrasoundImageProcessing::segmentationThread, this));
    setupNeuralNetworks();

    mImageTransform = Eigen::Affine3d::Identity();
}

UltrasoundImageProcessing::~UltrasoundImageProcessing() {
    {
        std::lock_guard<std::mutex> lock(mFrameBufferMutex);
        mStop = true;
    }
    mSegmentationThread->join();
}

void UltrasoundImageProcessing::execute() {
    auto port = getInputPort(0);

    auto cropper = UltrasoundImageCropper::New();
    cropper->setInputConnection(port);
    port = cropper->getOutputPort();
    cropper->update();

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

    mRawImage = getInputData<fast::Image>(0);
    mProcessedImage = port->getNextFrame<fast::Image>();

    AffineTransformation::pointer T = AffineTransformation::New();
    T->setTransform(mImageTransform.cast<float>());
    mProcessedImage->getSceneGraphNode()->setTransformation(T);

    try {
        addOutputData(0, mRawImage);
        addOutputData(1, mProcessedImage);
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

void UltrasoundImageProcessing::setImageTransform(Eigen::Affine3d transform){
    mImageTransform = transform;
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