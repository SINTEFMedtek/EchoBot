#include "UltrasoundInterface.hpp"

#include "FAST/Data/Image.hpp"

namespace fast {

UltrasoundInterface::UltrasoundInterface() {
    createInputPort<Image>(0);
    createOutputPort<Image>(0);
}

void UltrasoundInterface::execute() {
    mCurrentImage = getInputData<Image>(0);

    if(mRobotInterface->robot.isConnected())
        transformImageToProbeCenter();


    addOutputData(0, mCurrentImage);
}

void UltrasoundInterface::setRobotInterface(RobotInterfacePtr robotInterface)
{
    mRobotInterface = robotInterface;
}

void UltrasoundInterface::transformImageToProbeCenter(){
    Eigen::Affine3d offset = Eigen::Affine3d::Identity();
    Eigen::Vector3d translation((double)(mCurrentImage->getWidth()*mCurrentImage->getSpacing()(0)/2),0,-40);

    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX())*
        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());

    offset.translate(translation);
    offset.linear() = offset.linear()*m;

    Eigen::Affine3d rMb = mRobotInterface->robot.get_rMb();
    Eigen::Affine3d eeMt = mRobotInterface->robot.get_eeMt();
    Eigen::Affine3d bMee = mRobotInterface->robot.getCurrentState().getTransformToJoint(6);
    Eigen::Affine3d transform = rMb*bMee*eeMt*offset;

    AffineTransformation::pointer T = AffineTransformation::New();
    T->setTransform(transform.cast<float>());
    mCurrentImage->getSceneGraphNode()->setTransformation(T);
}

}