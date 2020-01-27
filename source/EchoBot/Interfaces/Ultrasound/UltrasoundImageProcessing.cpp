#include "UltrasoundImageProcessing.h"
#include "EchoBot/Core/Config.h"

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

    auto cropper = fast::UltrasoundImageCropper::New();
    cropper->setInputConnection(port);
    port = cropper->getOutputPort();
    cropper->update();
    mRawImage = port->getNextFrame<fast::Image>();

    if (mSegmentationEnabled) {
        mSegmentationNetwork->setInputConnection(port);
        port = mSegmentationNetwork->getOutputPort();
        mSegmentationNetwork->update();
    }

    mProcessedImage = port->getNextFrame<fast::Image>();

    AffineTransformation::pointer T = AffineTransformation::New();
    T->setTransform(mImageTransform.cast<float>());
    mProcessedImage->getSceneGraphNode()->setTransformation(T);

    try {
        addOutputData(0, mRawImage);
        addOutputData(1, mProcessedImage);
    } catch (fast::ThreadStopped &e) {
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
    mSegmentationNetwork = SegmentationNetwork::New();
    mSegmentationNetwork->setScaleFactor(1.0f / 255.0f);
    const auto engine = mSegmentationNetwork->getInferenceEngine()->getName();
    if(engine.substr(0,10) == "TensorFlow") {
        // TensorFlow needs to know what the output node is called
        mSegmentationNetwork->setOutputNode(0, "conv2d_23/truediv");
    }
    mSegmentationNetwork->load(fast::join(Config::getNeuralNetworkModelPath(), "aorta_segmentation_new.pb"));
    mSegmentationEnabled = true;
}

}