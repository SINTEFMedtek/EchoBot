#include "UltrasoundInterface.hpp"

#include "FAST/Algorithms/UltrasoundImageCropper/UltrasoundImageCropper.hpp"
#include "FAST/Algorithms/ImageCropper/ImageCropper.hpp"
#include "FAST/Algorithms/NeuralNetwork/PixelClassifier.hpp"
#include "FAST/Exporters/MetaImageExporter.hpp"
#include "FAST/Streamers/IGTLinkStreamer.hpp"
#include "FAST/Streamers/ClariusStreamer.hpp"

namespace echobot
{

UltrasoundInterface::UltrasoundInterface() {
}

UltrasoundInterface::~UltrasoundInterface() {
}

void UltrasoundInterface::connect()
{
    mProcessObject = UltrasoundImageProcessing::New();
    mProcessObject->setInputConnection(mUltrasoundStreamer->getOutputPort());
}

void UltrasoundInterface::setStreamer(UltrasoundStreamer streamer, std::string ip, uint32_t port)
{
    if(streamer == Clarius)
        mUltrasoundStreamer = ClariusStreamer::New();
    else if(streamer == IGTLink){
        auto usStreamer = IGTLinkStreamer::New();
        usStreamer->setConnectionAddress(ip);
        usStreamer->setConnectionPort(port);
        mUltrasoundStreamer = usStreamer;
    }
}

DataPort::pointer UltrasoundInterface::getOutputPort(uint portID)
{
    return mProcessObject->getOutputPort(portID);
}

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
    fast::Image::pointer input = getInputData<fast::Image>();

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

    if(mRecording)
    {
        MetaImageExporter::pointer imageExporter = MetaImageExporter::New();
        imageExporter->setInputData(input);
        imageExporter->setFilename(mStoragePath + "/Ultrasound/" + "US-2D_" + std::to_string(mFrameCounter) + ".mhd");
        imageExporter->update(0);

        ++mFrameCounter;
    }

    addOutputData(0, mCurrentImage);
    addOutputData(1, segmentation);
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

void UltrasoundImageProcessing::startRecording(std::string path) {
    mStoragePath = path;
    mFrameCounter = 0;
    mRecording = true;
    createDirectories((mStoragePath + "/Ultrasound"));
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