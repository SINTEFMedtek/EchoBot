//
// Created by androst on 06.01.20.
//

#ifndef ECHOBOT_POINTCLOUDUTILITIES_H
#define ECHOBOT_POINTCLOUDUTILITIES_H

#include "EchoBot/Core/SmartPointers.h"
#include "EchoBot/Core/DataTypes.h"

#include "FAST/ProcessObject.hpp"
#include "FAST/Data/Mesh.hpp"

namespace echobot {

Mesh::pointer decimateMesh(Mesh::pointer mesh, double fractionOfPointsToKeep);
Mesh::pointer reduceMeshExtent(Mesh::pointer mesh, float zMax, float zMin, float xMax, float xMin, float yMax, float yMin);
Vector3f calculateCentroid(std::vector<MeshVertex> vertices);

class MeshProcessing : public ProcessObject {
ECHOBOT_OBJECT(MeshProcessing)

public:
    void setBounds(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax);
    void setDecimationFraction(float fraction);

private:
    MeshProcessing();
    void execute();

    bool mBoundsModified = false;
    float m_zMax = std::numeric_limits<float>::max();
    float m_zMin = 0;
    float m_xMax = std::numeric_limits<float>::max();
    float m_xMin = -std::numeric_limits<float>::max();
    float m_yMax = std::numeric_limits<float>::max();
    float m_yMin = -std::numeric_limits<float>::max();
    float mDecimationFraction = 1.0;
};

}

#endif //ECHOBOT_POINTCLOUDUTILITIES_H
