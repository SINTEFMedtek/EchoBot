//
// Created by androst on 06.01.20.
//

#ifndef ECHOBOT_POINTCLOUDUTILITIES_H
#define ECHOBOT_POINTCLOUDUTILITIES_H

#include "FAST/Data/Mesh.hpp"

namespace echobot{
using namespace fast;

Mesh::pointer decimateMesh(Mesh::pointer mesh, double fractionOfPointsToKeep);
Mesh::pointer reduceMeshExtent(Mesh::pointer mesh, float zMax, float zMin, float xMax, float xMin, float yMax, float yMin);


}


#endif //ECHOBOT_POINTCLOUDUTILITIES_H
