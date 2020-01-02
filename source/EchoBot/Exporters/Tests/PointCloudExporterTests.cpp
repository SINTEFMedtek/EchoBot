//
// Created by androst on 06.08.19.
//

#include "EchoBot/Tests/catch.hpp"
#include "EchoBot/Exporters/PointCloudExporter.h"
#include "FAST/Data/Mesh.hpp"

using namespace echobot;
using namespace fast;

TEST_CASE("Export point cloud", "[EchoBot][Exporters]") {
        Mesh::pointer mesh = Mesh::New();
        std::vector<MeshVertex> vertices = {
                MeshVertex(Vector3f(1, 1, 1)),
                MeshVertex(Vector3f(1, 1, 10)),
                MeshVertex(Vector3f(1, 10, 10)),

                MeshVertex(Vector3f(1, 1, 1)),
                MeshVertex(Vector3f(1, 1, 10)),
                MeshVertex(Vector3f(30, 15, 15)),

                MeshVertex(Vector3f(1, 1, 10)),
                MeshVertex(Vector3f(1, 10, 10)),
                MeshVertex(Vector3f(30, 15, 15)),

                MeshVertex(Vector3f(1, 1, 1)),
                MeshVertex(Vector3f(1, 10, 10)),
                MeshVertex(Vector3f(30, 15, 15))
        };
        std::vector<MeshTriangle> triangles = {
                MeshTriangle(0, 1, 2),
                MeshTriangle(3, 4, 5),
                MeshTriangle(6, 7, 8),
                MeshTriangle(9, 10, 11)
        };

        mesh->create(vertices, {}, triangles);

        auto exporter = PointCloudExporter::New();
        exporter->setInputData(mesh);
        exporter->setFilename("VTKMeshFileExporter3DTest.vtk");
        CHECK_NOTHROW(exporter->update());
}