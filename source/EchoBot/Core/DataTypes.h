//
// Created by androst on 12.01.20.
//

#ifndef ECHOBOT_DATATYPES_H
#define ECHOBOT_DATATYPES_H

#include <FAST/ProcessObject.hpp>

#include <FAST/Data/Image.hpp>
#include <FAST/Data/Mesh.hpp>
#include <FAST/Data/Color.hpp>

#include <FAST/Streamers/Streamer.hpp>
#include <FAST/Streamers/RealSenseStreamer.hpp>
#include <FAST/Streamers/ImageFileStreamer.hpp>
#include <FAST/Streamers/MeshFileStreamer.hpp>

#include "FAST/Importers/ImageFileImporter.hpp"
#include "FAST/Importers/VTKMeshFileImporter.hpp"

#include "FAST/Algorithms/SurfaceExtraction/SurfaceExtraction.hpp"
#include "FAST/Algorithms/ImageResizer/ImageResizer.hpp"
#include <FAST/Algorithms/NeuralNetwork/SegmentationNetwork.hpp>

#include <FAST/Exporters/FileExporter.hpp>
#include <FAST/Exporters/MetaImageExporter.hpp>

#include <FAST/Visualization/Window.hpp>
#include <FAST/Visualization/View.hpp>
#include <FAST/Visualization/Renderer.hpp>
#include <FAST/Visualization/VertexRenderer/VertexRenderer.hpp>
#include <FAST/Visualization/ImageRenderer/ImageRenderer.hpp>
#include <FAST/Visualization/TriangleRenderer/TriangleRenderer.hpp>
#include <FAST/Visualization/SegmentationRenderer/SegmentationRenderer.hpp>

#include <romocc/Robot.h>

namespace echobot{
    using Eigen::Affine3f;
    using Eigen::Matrix3f;
    using Eigen::Vector3f;
    using Eigen::VectorXf;
    using Eigen::MatrixXf;

    using fast::ProcessObject;
    using fast::DataChannel;
    using fast::DataObject;
    using fast::Color;
    using fast::Image;
    using fast::AffineTransformation;
    using fast::Renderer;
    using fast::Streamer;
    using fast::Mesh;
    using fast::VertexRenderer;
    using fast::ImageRenderer;
    using fast::RealSenseStreamer;
    using fast::ImageFileStreamer;
    using fast::MeshFileStreamer;
    using fast::MeshVertex;
    using fast::MeshTriangle;
    using fast::TriangleRenderer;
    using fast::Window;
    using fast::View;
    using fast::FileExporter;
    using fast::MetaImageExporter;
    using fast::ImageFileImporter;
    using fast::ImageResizer;
    using fast::SurfaceExtraction;
    using fast::VTKMeshFileImporter;
    using fast::SegmentationNetwork;
    using fast::SegmentationRenderer;

    using romocc::TransformUtils::Affine::toVector6D;
    using romocc::TransformUtils::Affine::toAffine3DFromVector6D;
}

#endif //ECHOBOT_DATATYPES_H
