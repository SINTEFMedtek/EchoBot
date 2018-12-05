#ifndef ROBOTVISUALIZATION_H
#define ROBOTVISUALIZATION_H

#include "FAST/ProcessObject.hpp"
#include "FAST/Visualization/TriangleRenderer/TriangleRenderer.hpp"

#include "../RobotInterface.h"

/**
 * Implementation of visualization of Robot Manipulator and tool.
 *
 * \author Andreas Østvik
 */

using namespace fast;

class RobotPart
{
public:
    RobotPart();
    RobotPart(std::string filename);

    ~RobotPart(){};

    void transform(Eigen::Affine3d transform);
    void rotate(double x, double y, double z);

    void setMeshFile(std::string filename);
    void setTransformation(Eigen::Affine3d transform);

    Mesh::pointer getMesh();

private:
    Mesh::pointer mMesh;
    Mesh::pointer getMeshFromFile(std::string filename);

};

class RobotTool : public RobotPart
{
public:
    RobotTool();
    RobotTool(std::string filename);

    ~RobotTool(){};

    void setMeshFile(std::string filename);

    TriangleRenderer::pointer getRenderer();
private:
    TriangleRenderer::pointer mRenderer;
};

class RobotManipulator
{

public:
    RobotManipulator();
    ~RobotManipulator(){};

    void setInterface(RobotInterfacePtr robotInterface);

    TriangleRenderer::pointer getRenderer();
    RobotTool getTool(){ return mTool;};

private:
    TriangleRenderer::pointer mRenderer;

    void addPart(RobotPart part);
    std::vector<RobotPart> mParts;

    RobotTool mTool;

    void updatePositions();

    RobotInterfacePtr mRobotInterface;
};




#endif // ROBOTVISUALIZATION_H
