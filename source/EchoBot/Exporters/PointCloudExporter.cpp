//
// Created by androst on 20.12.19.
//

#include "PointCloudExporter.h"
#include "FAST/Data/Mesh.hpp"
#include "FAST/SceneGraph.hpp"
#include <fmt/format.h>


namespace echobot {


PointCloudExporter::PointCloudExporter() {
    createInputPort<Mesh>(0);
    mWriteNormals = false;
    mWriteColors = false;
}

void PointCloudExporter::setWriteNormals(bool writeNormals) {
    mWriteNormals = writeNormals;
}

void PointCloudExporter::setWriteColors(bool writeColors)  {
    mWriteColors = writeColors;
}

void PointCloudExporter::execute() {
    if(mFilename == "")
        throw fast::Exception("No filename given to the PointCloudExporter");

    auto mesh = getInputData<Mesh>();

    // Get transformation
    auto transform = fast::SceneGraph::getAffineTransformationFromData(mesh)->getTransform();

    FILE *file;
    file = fopen(mFilename.c_str(), "w");

    if(!file)
        throw fast::Exception("Unable to open the file " + mFilename);

    fmt::memory_buffer buffer;

    // Write header
    fmt::format_to(buffer, "# vtk DataFile Version 3.0\n");
    fmt::format_to(buffer, "vtk output\n");
    fmt::format_to(buffer, "ASCII\n", "");
    fmt::format_to(buffer, "DATASET POLYDATA\n");

    // Write vertices
    auto access = mesh->getMeshAccess(ACCESS_READ);
    auto vertices = access->getVertices();

    auto start = std::chrono::high_resolution_clock::now();

    fmt::format_to(buffer, "POINTS {} float\n", vertices.size());
    for(auto vertex : vertices) {
        auto point = (transform.matrix() * vertex.getPosition().homogeneous()).head(3);
        fmt::format_to(buffer, "{0} {1} {2}\n", point.x(), point.y(), point.z());
    }

    if (mesh->getNrOfTriangles() > 0) {
        std::vector<fast::MeshTriangle> triangles = access->getTriangles();
        // Write triangles

        fmt::format_to(buffer, "POLYGONS {0} {1}\n", mesh->getNrOfTriangles(), mesh->getNrOfTriangles() * 4);
        for (auto triangle : triangles) {
            fmt::format_to(buffer, "3 {0} {1} {2}\n", triangle.getEndpoint1(), triangle.getEndpoint2(),
                           triangle.getEndpoint3());
        }

    }

    if (mesh->getNrOfLines() > 0) {
        // Write lines
        auto lines = access->getLines();
        fmt::format_to(buffer, "LINES {0} {1}\n", mesh->getNrOfLines(), mesh->getNrOfLines() * 3);

        for (auto line : lines) {
            fmt::format_to(buffer, "2 {0} {1}\n", line.getEndpoint1(), line.getEndpoint2());
        }
    }

    if (mWriteNormals) {
        fmt::format_to(buffer, "POINTS_DATA {}\n", vertices.size());
        fmt::format_to(buffer, "NORMALS Normals float\n");

        for (auto vertex : vertices) {
            VectorXf normal = vertex.getNormal();
            normal = transform.linear() * normal; // Transform the normal

            // Normalize it
            float length = normal.norm();
            if (length == 0) { // prevent NaN situations
                fmt::format_to(buffer, "0 1 0\n");
            } else {
                normal.normalize();
                fmt::format_to(buffer, "{0} {1} {2}\n", normal.x(), normal.y(), normal.z());
            }
            auto normals_part_stop = std::chrono::high_resolution_clock::now();
        }
    }

    if (mWriteColors) {
        fmt::format_to(buffer, "POINTS_DATA {}\n", vertices.size());
        fmt::format_to(buffer, "VECTORS vertex_colors float\n");

        for (auto vertex : vertices) {
            Color color = vertex.getColor();
            fmt::format_to(buffer, "{0} {1} {2}\n", color.getRedValue(), color.getGreenValue(),
                           color.getBlueValue());
        }
    }

        std::string out = fmt::to_string(buffer);
        fwrite(out.c_str(), 1, out.length(), file);
        fclose(file);
    }

}
