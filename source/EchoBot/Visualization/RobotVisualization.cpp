#include "RobotVisualization.h"

#include <FAST/Importers/VTKMeshFileImporter.hpp>
#include "FAST/SceneGraph.hpp"

namespace echobot
{

RobotVisualizator::RobotVisualizator()
{
    std::string CADModelPath = "/home/androst/dev/ROMO/EchoBot/source/EchoBot/Visualization/CADModels/";

    this->addPart("base", CADModelPath + "base.vtk");
    this->addPart("shoulder", CADModelPath + "shoulder.vtk");
    this->addPart("forearm", CADModelPath + "forearm.vtk");
    this->addPart("upperarm", CADModelPath + "upperarm.vtk");
    this->addPart("wrist1", CADModelPath + "wrist1.vtk");
    this->addPart("wrist2", CADModelPath + "wrist2.vtk");
    this->addPart("wrist3", CADModelPath + "wrist3.vtk");
    this->addTool("tool", CADModelPath + "5S-Probe.vtk");

    mRenderer = TriangleRenderer::New();
}

void RobotVisualizator::setInterface(RobotInterface::pointer robotInterface)
{
    mRobotInterface = robotInterface;
    QObject::connect(mRobotInterface.get(), &RobotInterface::stateUpdated,
            std::bind(&RobotVisualizator::updatePositions, this));
}

void RobotVisualizator::updatePositions()
{
    std::lock_guard<std::mutex> lock(mUpdateMutex);
    romocc::Transform3d rMb = mRobotInterface->robot->getCoordinateSystem()->get_rMb();
    romocc::Transform3d eeMt = mRobotInterface->robot->getCoordinateSystem()->get_eeMt();
    romocc::RobotState::pointer currentState = mRobotInterface->robot->getCurrentState();

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

    Eigen::Affine3d transformFixProbe = Eigen::Affine3d::Identity();

    Eigen::Matrix3d rotProbe;
    rotProbe = Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ())*
               Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())*
               Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());
    transformFixProbe.linear() = transformFixProbe.linear()*rotProbe;

    mTool->setTransformation(rMb*currentState->getTransformToJoint(6)*eeMt*transformFixProbe);
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
    mRenderer->addInputData(mParts["forearm"]->getMesh());
    mRenderer->addInputData(mParts["upperarm"]->getMesh());
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


void RobotPart::rotate(double x, double y, double z)
{
    AffineTransformation::pointer T = mMesh->getSceneGraphNode()->getTransformation();

    Matrix3f m;
    m = Eigen::AngleAxisf(z*M_PI/180, Vector3f::UnitZ())*
        Eigen::AngleAxisf(x*M_PI/180, Vector3f::UnitX())*
        Eigen::AngleAxisf(y*M_PI/180, Vector3f::UnitY());

    Affine3f transform = T->getTransform();
    transform.linear() = transform.linear()*m;

    T->setTransform(transform);
    mMesh->getSceneGraphNode()->setTransformation(T);
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

    fast::DataPort::pointer importPort = importer->getOutputPort();
    importer->update(0);

    return importPort->getNextFrame<fast::Mesh>();
}



}