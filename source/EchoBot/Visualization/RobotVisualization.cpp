#include "RobotVisualization.h"
#include "EchoBot/Core/DataTypes.h"
#include <FAST/Importers/VTKMeshFileImporter.hpp>
#include "FAST/SceneGraph.hpp"

namespace echobot
{

RobotVisualizator::RobotVisualizator()
{
    mCADModelPath = "../../source/EchoBot/Visualization/CADModels/";
    mRenderer = TriangleRenderer::New();
}

void RobotVisualizator::setInterface(RobotInterface::pointer robotInterface)
{
    mRobotInterface = robotInterface;

    auto subpath = "ur5/";
    auto probe_filename = "clarius_probe_with_holder_ur5.vtk";

    if(mRobotInterface->getManipulatorInfo().manipulator == Manipulator::UR10)
    {
        subpath = "ur10e/";
        probe_filename = "clarius_probe_with_holder_ur10.vtk";
    }

    this->addPart("base", mCADModelPath + subpath + "base.vtk");
    this->addPart("shoulder", mCADModelPath + subpath + "shoulder.vtk");
    this->addPart("forearm", mCADModelPath + subpath + "forearm.vtk");
    this->addPart("upperarm", mCADModelPath + subpath +"upperarm.vtk");
    this->addPart("wrist1", mCADModelPath + subpath +"wrist1.vtk");
    this->addPart("wrist2", mCADModelPath + subpath +"wrist2.vtk");
    this->addPart("wrist3", mCADModelPath + subpath +"wrist3.vtk");
    this->addTool("tool", mCADModelPath + probe_filename);

    QObject::connect(mRobotInterface.get(), &RobotInterface::stateUpdated,
            std::bind(&RobotVisualizator::updatePositions, this));
}

void RobotVisualizator::updatePositions()
{
    auto currentState = mRobotInterface->getRobot()->getCurrentState();
    auto rMb = mRobotInterface->getRobot()->getCoordinateSystem()->get_rMb();
    auto eeMt = mRobotInterface->getRobot()->getCoordinateSystem()->get_eeMt();

    if(currentState->getJointConfig() != mPreviousJointConfig
    || rMb.matrix() != mPrevious_rMb.matrix()
    || eeMt.matrix() != mPrevious_eeMt.matrix())
    {
        if(mRobotInterface->getManipulatorInfo().manipulator == Manipulator::UR5)
        {
            Eigen::Affine3d offset_link2 = Eigen::Affine3d::Identity();
            offset_link2.translate(Eigen::Vector3d(0.0,0.0,121.0));

            mParts["base"]->setTransformation(rMb);
            mParts["shoulder"]->setTransformation(rMb*currentState->getTransformToJoint(1));
            mParts["forearm"]->setTransformation(rMb*currentState->getTransformToJoint(1)*offset_link2);
            mParts["forearm"]->rotate(0,0, currentState->getJointConfig()(1)*180/M_PI+90);
            mParts["upperarm"]->setTransformation(rMb*currentState->getTransformToJoint(2));
            mParts["upperarm"]->rotate(0,0, currentState->getJointConfig()(2)*180/M_PI);
            mParts["wrist1"]->setTransformation(rMb*currentState->getTransformToJoint(4));
            mParts["wrist2"]->setTransformation(rMb*currentState->getTransformToJoint(5));
            mParts["wrist3"]->setTransformation(rMb*currentState->getTransformToJoint(6));
            mTool->setTransformation(rMb*currentState->getTransformToJoint(6)*eeMt);
        } else if(mRobotInterface->getManipulatorInfo().manipulator == Manipulator::UR10)
        {
            mParts["base"]->setTransformation(rMb*currentState->getTransformToJoint(0));
            mParts["base"]->rotate(90, 0, 0);

            mParts["shoulder"]->setTransformation(rMb*currentState->getTransformToJoint(1));

            mParts["upperarm"]->setTransformation(rMb*currentState->getTransformToJoint(1));
            mParts["upperarm"]->rotate(0, 0, currentState->getJointConfig()(1)*180/M_PI+90);

            mParts["forearm"]->setTransformation(rMb*currentState->getTransformToJoint(2));
            mParts["forearm"]->rotate(0, 0, currentState->getJointConfig()(2)*180/M_PI+90);

            mParts["wrist1"]->setTransformation(rMb*currentState->getTransformToJoint(4));
            mParts["wrist2"]->setTransformation(rMb*currentState->getTransformToJoint(5));
            mParts["wrist3"]->setTransformation(rMb*currentState->getTransformToJoint(6));

            mTool->setTransformation(rMb*currentState->getTransformToJoint(6)*eeMt);
        }

        mPreviousJointConfig = currentState->getJointConfig();
        mPrevious_rMb = rMb;
        mPrevious_eeMt = eeMt;
    }

}

void RobotPart::setTransformation(Eigen::Affine3d transform)
{
    AffineTransformation::pointer T = AffineTransformation::New();
    T->setTransform(transform.cast<float>());
    mMesh->getSceneGraphNode()->setTransformation(T);
}

void RobotVisualizator::addPart(std::string partName, std::string cadFilepath)
{
    auto part = RobotPart::New();
    part->setPart(partName, cadFilepath);
    mParts[partName] = part;
}

void RobotVisualizator::addTool(std::string toolName, std::string cadFilepath)
{
    auto tool = RobotTool::New();
    tool->setPart(toolName, cadFilepath);
    mTool = tool;
}


TriangleRenderer::pointer RobotVisualizator::getRenderer()
{
    mRenderer = TriangleRenderer::New();
    mRenderer->addInputData(mParts["base"]->getMesh());
    mRenderer->addInputData(mParts["shoulder"]->getMesh());
    mRenderer->addInputData(mParts["upperarm"]->getMesh());
    mRenderer->addInputData(mParts["forearm"]->getMesh());
    mRenderer->addInputData(mParts["wrist1"]->getMesh());
    mRenderer->addInputData(mParts["wrist2"]->getMesh());
    mRenderer->addInputData(mParts["wrist3"]->getMesh());
    return mRenderer;
}

RobotTool::RobotTool()
{
}

TriangleRenderer::pointer RobotTool::getRenderer()
{
    mRenderer = fast::TriangleRenderer::New();
    mRenderer->addInputData(this->getMesh());
    return mRenderer;
}

RobotPart::RobotPart()
{
}

void RobotPart::setPart(std::string partName, std::string filename)
{
    mPartName = partName;
    mMesh = getMeshFromFile(filename);
}

Mesh::pointer RobotPart::getMesh ()
{
    return mMesh;
}

std::string RobotPart::getName() const
{
    return mPartName;
}


void RobotPart::rotate(double rx, double ry, double rz)
{
    AffineTransformation::pointer T = mMesh->getSceneGraphNode()->getTransformation();

    Matrix3f m;
    m = Eigen::AngleAxisf(rx*M_PI/180, Vector3f::UnitX())*
        Eigen::AngleAxisf(ry*M_PI/180, Vector3f::UnitY())*
        Eigen::AngleAxisf(rz*M_PI/180, Vector3f::UnitZ());

    Affine3f transform = T->getTransform();
    transform.linear() = transform.linear()*m;

    T->setTransform(transform);
    mMesh->getSceneGraphNode()->setTransformation(T);
}

void RobotPart::translate(double x, double y, double z)
{
    auto currentTransform = mMesh->getSceneGraphNode()->getTransformation();
    auto offset = Eigen::Affine3f::Identity();
    offset.translate(Eigen::Vector3f(x, y, z));
    Affine3f newTransform = currentTransform->getTransform()*offset;
    currentTransform->setTransform(newTransform);
    mMesh->getSceneGraphNode()->setTransformation(currentTransform);
}

void RobotPart::transform(Eigen::Affine3d transform)
{
    AffineTransformation::pointer T = mMesh->getSceneGraphNode()->getTransformation();

    Affine3f current = T->getTransform();
    T->setTransform(current*transform.cast<float>());

    mMesh->getSceneGraphNode()->setTransformation(T);
}

Mesh::pointer RobotPart::getMeshFromFile(std::string filename)
{
    fast::VTKMeshFileImporter::pointer importer = fast::VTKMeshFileImporter::New();
    importer->setFilename(filename);

    DataChannel::pointer importPort = importer->getOutputPort();
    importer->update();

    return importPort->getNextFrame<fast::Mesh>();
}



}