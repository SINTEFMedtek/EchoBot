#include "robotVisualization.h"

#include <FAST/Importers/VTKMeshFileImporter.hpp>
#include "FAST/SceneGraph.hpp"

RobotManipulator::RobotManipulator()
{
    std::string CADModelPath = "AorticAneurysmExam/visualization/CADModels/with_normals_new/";

    RobotPart base = RobotPart(CADModelPath + "base.vtk");
    RobotPart shoulder = RobotPart(CADModelPath + "shoulder.vtk");
    RobotPart forearm = RobotPart(CADModelPath + "forearm.vtk");
    RobotPart upperarm = RobotPart(CADModelPath + "upperarm.vtk");
    RobotPart wrist1 = RobotPart(CADModelPath + "wrist1.vtk");
    RobotPart wrist2 = RobotPart(CADModelPath + "wrist2.vtk");
    RobotPart wrist3 = RobotPart(CADModelPath + "wrist3.vtk");

    mRenderer = fast::TriangleRenderer::New();

    addPart(base);
    addPart(shoulder);
    addPart(forearm);
    addPart(upperarm);
    addPart(wrist1);
    addPart(wrist2);
    addPart(wrist3);

    mTool = RobotTool(CADModelPath + "5S-Probe.vtk");
}

void RobotManipulator::setInterface(RobotInterfacePtr robotInterface)
{
    mRobotInterface = robotInterface;
    QObject::connect(&mRobotInterface->robot, &corah::Robot::stateUpdated, std::bind(&RobotManipulator::updatePositions, this));
}

void RobotManipulator::updatePositions()
{
    corah::Transform3d rMb = mRobotInterface->robot.get_rMb();
    corah::Transform3d eeMt = mRobotInterface->robot.get_eeMt();

    corah::RobotState currentState = mRobotInterface->robot.getCurrentState();

    Eigen::Vector3d translation(0.0,0.0,121.0);
    Eigen::Affine3d offset_link2 = Eigen::Affine3d::Identity();
    offset_link2.translate(translation);

    mParts[0].setTransformation(rMb);
    mParts[1].setTransformation(rMb*currentState.getTransformToJoint(1));
    mParts[2].setTransformation(rMb*currentState.getTransformToJoint(1)*offset_link2);
    mParts[2].rotate(0,0, currentState.jointConfiguration(1)*180/M_PI+90);
    mParts[3].setTransformation(rMb*currentState.getTransformToJoint(2));
    mParts[3].rotate(0,0, currentState.jointConfiguration(2)*180/M_PI);
    mParts[4].setTransformation(rMb*currentState.getTransformToJoint(4));
    mParts[5].setTransformation(rMb*currentState.getTransformToJoint(5));
    mParts[6].setTransformation(rMb*currentState.getTransformToJoint(6));

    mTool.setTransformation(rMb*currentState.getTransformToJoint(6)*eeMt);
}

void RobotManipulator::addPart(RobotPart part)
{
    mParts.push_back(part);
    mRenderer->addInputData(part.getMesh());
}

TriangleRenderer::pointer RobotManipulator::getRenderer()
{
    return mRenderer;
}

RobotPart::RobotPart()
{
}

RobotPart::RobotPart(std::string filename)
{
    this->setMeshFile(filename);
}


void RobotPart::setMeshFile(std::string filename)
{
    mMesh = getMeshFromFile(filename);
}

Mesh::pointer RobotPart::getMesh()
{
    return mMesh;
}

void RobotPart::setTransformation(Eigen::Affine3d transform)
{
    AffineTransformation::pointer T = AffineTransformation::New();
    T->setTransform(transform.cast<float>());

    mMesh->getSceneGraphNode()->setTransformation(T);
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

RobotTool::RobotTool():
        RobotPart()
{
}

RobotTool::RobotTool(std::string filename):
    RobotPart(filename)
{
    mRenderer = fast::TriangleRenderer::New();
    mRenderer->setOpacity(0,1.0);
    mRenderer->addInputData(this->getMesh());
}

TriangleRenderer::pointer RobotTool::getRenderer()
{
    return mRenderer;
}