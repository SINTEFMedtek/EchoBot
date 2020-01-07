//
// Created by androst on 06.01.20.
//

#include "PointCloudUtilities.h"
#include <random>

namespace echobot{

Mesh::pointer decimateMesh(Mesh::pointer pointCloud, double fractionOfPointsToKeep) {
    MeshAccess::pointer accessFixedSet = pointCloud->getMeshAccess(ACCESS_READ);
    std::vector<MeshVertex> vertices = accessFixedSet->getVertices();

    // Sample the preferred amount of points from the point cloud
    auto numVertices = (unsigned int) vertices.size();
    auto numSamplePoints = (unsigned int) ceil(fractionOfPointsToKeep * numVertices);
    std::vector<MeshVertex> newVertices;

    std::unordered_set<int> movingIndices;
    unsigned int sampledPoints = 0;
    std::default_random_engine distributionEngine;
    std::uniform_int_distribution<unsigned int> distribution(0, numVertices - 1);
    while (sampledPoints < numSamplePoints) {
        unsigned int index = distribution(distributionEngine);
        if (movingIndices.count(index) < 1 && vertices.at(index).getPosition().array().isNaN().sum() == 0) {
            newVertices.push_back(vertices.at(index));
            movingIndices.insert(index);
            ++sampledPoints;
        }
    }

    // Add noise to point cloud
    float minX, minY, minZ;
    Vector3f position0 = vertices[0].getPosition();
    minX = position0[0];
    minY = position0[1];
    minZ = position0[2];
    float maxX = minX, maxY = minY, maxZ = minZ;
    for (auto &vertex : vertices) {
        Vector3f position = vertex.getPosition();
        if (position[0] < minX) { minX = position[0]; }
        if (position[0] > maxX) { maxX = position[0]; }
        if (position[1] < minY) { minY = position[1]; }
        if (position[1] > maxY) { maxY = position[1]; }
        if (position[2] < minZ) { minZ = position[2]; }
        if (position[2] > maxZ) { maxZ = position[2]; }
    }
    Mesh::pointer newCloud = Mesh::New();
    newCloud->create(newVertices);
    // Update point cloud to include the removed points and added noise
    return newCloud;
}

Mesh::pointer reduceMeshExtent(Mesh::pointer mesh, float zMin, float zMax, float xMin, float xMax, float yMin, float yMax) {
    auto meshAccess = mesh->getMeshAccess(ACCESS_READ);
    std::vector<MeshVertex> vertices = meshAccess->getVertices();
    std::vector<MeshVertex> filteredPoints;

    for (auto vertex: vertices) {
        auto position = vertex.getPosition();
        if (!position.z() || position.z() > zMax || position.z() < zMin)
            continue;

        if (position.x() > xMax || position.x() < xMin)
            continue;

        if (position.y() > yMax || position.y() < yMin)
            continue;

        filteredPoints.push_back(vertex);
    }

    auto reducedMesh = Mesh::New();
    reducedMesh->create(filteredPoints);
    reducedMesh->setCreationTimestamp(mesh->getCreationTimestamp());
    return reducedMesh;
}

}
