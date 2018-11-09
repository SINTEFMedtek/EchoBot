#ifndef ROBOTVISUALIZATION_H
#define ROBOTVISUALIZATION_H

#include "FAST/ProcessObject.hpp"
#include "FAST/Visualization/TriangleRenderer/TriangleRenderer.hpp"

#include "../RobotInterface.h"

/**
 * Implementation of visualization of Robot Manipulator.
 *
 * \author Andreas Østvik
 */

using namespace fast;

class RobotPart
{
public:
    RobotPart(std::string filename);
    ~RobotPart(){};

    void transform(Eigen::Affine3d transform);
    void rotate(double x, double y, double z);

    void setTransformation(Eigen::Affine3d transform);

    Mesh::pointer getMesh();
    TriangleRenderer::pointer getRenderer();
private:
    TriangleRenderer::pointer mRenderer;
    Mesh::pointer mMesh;

    Mesh::pointer getMeshFromFile(std::string filename);

};


class RobotTool : public RobotPart
{
public:
    RobotTool(std::string filename) : RobotPart(filename){};
    ~RobotTool(){};
};

class RobotManipulator
{

public:
    RobotManipulator(RobotInterfacePtr robotInterface);
    ~RobotManipulator();

    TriangleRenderer::pointer getRenderer(uint linknr = 0);

private:
    std::vector<RobotPart> mParts;
    void updatePositions();

    RobotInterfacePtr mRobotInterface;

};




#endif // ROBOTVISUALIZATION_H
