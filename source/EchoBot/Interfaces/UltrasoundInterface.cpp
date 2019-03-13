#include "UltrasoundInterface.hpp"

#include "FAST/Algorithms/UltrasoundImageCropper/UltrasoundImageCropper.hpp"
#include "FAST/Algorithms/ImageCropper/ImageCropper.hpp"
#include "FAST/Algorithms/NeuralNetwork/PixelClassifier.hpp"

using namespace fast;

UltrasoundInterface::UltrasoundInterface() {
    createInputPort<Image>(0);

    createOutputPort<Image>(0);
    createOutputPort<Image>(1); // Segmentation

    mSegmentationThread = new std::thread(std::bind(&UltrasoundInterface::segmentationThread, this));
    setupNeuralNetworks();
}

UltrasoundInterface::~UltrasoundInterface() {
    std::cout << "Stopping segmentation thread.." << std::endl;
    {
        std::lock_guard<std::mutex> lock(mFrameBufferMutex);
        mStop = true;
    }
    mSegmentationThread->join();
    std::cout << "Segmentation thread stopped!" << std::endl;
}

void UltrasoundInterface::execute() {
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

    addOutputData(0, mCurrentImage);
    addOutputData(1, segmentation);
}

void UltrasoundInterface::setRobotInterface(RobotInterfacePtr robotInterface) {
    mRobotInterface = robotInterface;
}

void UltrasoundInterface::transformImageToProbeCenter() {
    Eigen::Affine3d offset = Eigen::Affine3d::Identity();

    Eigen::Vector3d translation((double) (mCurrentImage->getWidth() * mCurrentImage->getSpacing()(0) / 2), 0,
                                0); // z=-40

    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());

    offset.translate(translation);
    offset.linear() = offset.linear() * m;

    Eigen::Affine3d rMb = mRobotInterface->robot->get_rMb();
    Eigen::Affine3d eeMt = mRobotInterface->robot->get_eeMt();
    Eigen::Affine3d bMee = mRobotInterface->robot->getCurrentState().getTransformToJoint(6);
    Eigen::Affine3d transform = rMb * bMee * eeMt * offset;

    AffineTransformation::pointer T = AffineTransformation::New();
    T->setTransform(transform.cast<float>());
    mCurrentImage->getSceneGraphNode()->setTransformation(T);
}

void UltrasoundInterface::segmentationThread() {
    while (true) {
        {
            std::lock_guard<std::mutex> lock(mFrameBufferMutex);
            if (mStop)
                break;
        }
    }
}

void UltrasoundInterface::setupNeuralNetworks() {
//    mPixelClassifier = PixelClassifier::New();
//    mPixelClassifier->setNrOfClasses(2);
//    mPixelClassifier->setResizeBackToOriginalSize(false);
//    mPixelClassifier->load("/home/androst/Data/NNModels/phantom.pb");
//    mPixelClassifier->setScaleFactor(1.0f / 255.0f);
//    mPixelClassifier->addOutputNode(0, "conv2d_23/truediv");
}