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
    ECHOBOT_OBJECT(RobotPart)

    public:
        RobotPart();

        void transform(Eigen::Affine3d transform);
        void rotate(double x, double y, double z);

        void setPart(std::string partName, std::string cadFile);
        void setTransformation(Eigen::Affine3d transform);

        Mesh::pointer getMesh();
        std::string getName() const;


    private:
        Mesh::pointer mMesh;
        std::string mPartName;
        Mesh::pointer getMeshFromFile(std::string filename);

        std::weak_ptr<RobotPart> mPtr;
};

class RobotTool : public RobotPart
{
    ECHOBOT_OBJECT(RobotTool)

    public:
        RobotTool();
        TriangleRenderer::pointer getRenderer();

    private:
        TriangleRenderer::pointer mRenderer;
        std::weak_ptr<RobotTool> mPtr;
};

class RobotVisualizator
{
    ECHOBOT_OBJECT(RobotVisualizator)

    public:
        RobotVisualizator();
        void setInterface(RobotInterface::pointer robotInterface);

        TriangleRenderer::pointer getRenderer();
        RobotTool::pointer getTool(){ return this->mTool;};

    private:
        RobotInterface::pointer mRobotInterface;
        TriangleRenderer::pointer mRenderer;
        std::map<std::string, RobotPart::pointer> mParts;
        RobotTool::pointer mTool;

        void addPart(std::string partName, std::string cadFilepath);
        void addTool(std::string toolName, std::string cadFilepath);

        void updatePositions();

        std::weak_ptr<RobotVisualizator> mPtr;
};

}

#endif // ROBOTVISUALIZATION_H
