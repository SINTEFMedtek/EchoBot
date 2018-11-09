#include "robotVisualization.h"

#include <FAST/Importers/VTKMeshFileImporter.hpp>
#include "FAST/SceneGraph.hpp"

RobotManipulator::RobotManipulator(RobotInterfacePtr robotInterface):
    mRobotInterface(robotInterface)
{
    RobotPart base = RobotPart("AorticAneurysmExam/visualization/CADModels/base.vtk");
    RobotPart link1 = RobotPart("AorticAneurysmExam/visualization/CADModels/link1.vtk");
    RobotPart link2 = RobotPart("AorticAneurysmExam/visualization/CADModels/link2.vtk");
    RobotPart link3 = RobotPart("AorticAneurysmExam/visualization/CADModels/link3.vtk");
    RobotPart link4 = RobotPart("AorticAneurysmExam/visualization/CADModels/link4.vtk");
    RobotPart link5 = RobotPart("AorticAneurysmExam/visualization/CADModels/link5.vtk");
    RobotPart endeffector = RobotPart("AorticAneurysmExam/visualization/CADModels/endeffector.vtk");

    mParts.push_back(base);
    mParts.push_back(link1);
    mParts.push_back(link2);
    mParts.push_back(link3);
    mParts.push_back(link4);
    mParts.push_back(link5);
    mParts.push_back(endeffector);

    QObject::connect(&mRobotInterface->robot, &corah::Robot::stateUpdated, std::bind(&RobotManipulator::updatePositions, this));
}

void RobotManipulator::updatePositions()
{
    corah::Transform3d rMb = mRobotInterface->robot.get_rMb();
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
}

TriangleRenderer::pointer RobotManipulator::getRenderer(uint linknr)
{
    return mParts[linknr].getRenderer();
}

RobotPart::RobotPart(std::string filename)
{
    mMesh = getMeshFromFile(filename);

    mRenderer = fast::TriangleRenderer::New();
    mRenderer->setDefaultColor(fast::Color::White());
    mRenderer->addInputData(mMesh);
}

TriangleRenderer::pointer RobotPart::getRenderer()
{
    return mRenderer;
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