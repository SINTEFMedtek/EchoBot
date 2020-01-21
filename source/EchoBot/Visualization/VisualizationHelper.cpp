//
// Created by androst on 16.01.20.
//

#include "VisualizationHelper.h"

#include "EchoBot/Core/DataTypes.h"
#include <FAST/Visualization/LineRenderer/LineRenderer.hpp>

namespace echobot {

fast::LineRenderer::pointer VisualizationHelper::createCoordinateFrameRenderer(Eigen::Affine3f transform,
        float axisLength)
{
    std::vector<MeshVertex> vertices = {
            MeshVertex(transform.linear()*Vector3f(0, 0, 0) + transform.translation()),
            MeshVertex(transform.linear()*Vector3f(axisLength, 0, 0) + transform.translation()),
            MeshVertex(transform.linear()*Vector3f(0, axisLength, 0) + transform.translation()),
            MeshVertex(transform.linear()*Vector3f(0, 0, axisLength) + transform.translation()),
    };


    std::vector<fast::MeshLine> x_line = {fast::MeshLine(0, 1)};
    std::vector<fast::MeshLine> y_line = {fast::MeshLine(0, 2)};
    std::vector<fast::MeshLine> z_line = {fast::MeshLine(0, 3)};

    auto x_mesh = Mesh::New();
    x_mesh->create(vertices, x_line);

    auto y_mesh = Mesh::New();
    y_mesh->create(vertices, y_line);

    auto z_mesh = Mesh::New();
    z_mesh->create(vertices, z_line);

    auto lineRenderer = fast::LineRenderer::New();
    lineRenderer->addInputData(x_mesh);
    lineRenderer->addInputData(y_mesh);
    lineRenderer->addInputData(z_mesh);
    lineRenderer->setColor(0, Color::Red());
    lineRenderer->setColor(1, Color::Green());
    lineRenderer->setColor(2, Color::Blue());
    return lineRenderer;
}

}
