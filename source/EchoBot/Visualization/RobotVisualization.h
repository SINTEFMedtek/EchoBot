#ifndef ROBOTVISUALIZATION_H
#define ROBOTVISUALIZATION_H

#include "EchoBot/Interfaces/RobotInterface.h"

#include "FAST/ProcessObject.hpp"
#include "FAST/Visualization/TriangleRenderer/TriangleRenderer.hpp"

/**
 * Implementation of visualization of Robot Visualizator and tool.
 *
 * \author Andreas Østvik
 */

namespace echobot
{
using namespace fast;

class RobotPart
{
public:
    RobotPart();
    RobotPart(std::string name, std::string meshfile);

    ~RobotPart(){};

    void transform(Eigen::Affine3d transform);
    void rotate(double x, double y, double z);

    void setMeshFile(std::string filename);
    void setTransformation(Eigen::Affine3d transform);

    Mesh::pointer getMesh();
    std::string getName() const;


private:
    Mesh::pointer mMesh;
    Mesh::pointer getMeshFromFile(std::string filename);

    std::string mPartName;

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

class RobotVisualizator
{
    ECHOBOT_OBJECT(RobotVisualizator)

    public:
        RobotVisualizator();
        ~RobotVisualizator(){};

        void setInterface(RobotInterface::pointer robotInterface);

        TriangleRenderer::pointer getRenderer();
        RobotTool getTool(){ return mTool;};

    private:
        RobotInterface::pointer mRobotInterface;
        TriangleRenderer::pointer mRenderer;
        std::map<std::string, RobotPart> mParts;
        RobotTool mTool;

        void addPart(RobotPart part);
        void updatePositions();

        std::weak_ptr<RobotVisualizator> mPtr;
};

}

#endif // ROBOTVISUALIZATION_H
