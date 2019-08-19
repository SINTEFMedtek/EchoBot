#include "RecordTool.h"

#include <FAST/Exporters/VTKMeshFileExporter.hpp>
#include <FAST/Exporters/MetaImageExporter.hpp>

namespace echobot {

void RecordTool::addRecordChannel(std::string name, DataChannel::pointer channel)
{
    mRecordChannels[name] = channel;

    uint port = getNrOfInputConnections();
    createInputPort<DataObject>(port);
    createOutputPort<DataObject>(port);
    setInputConnection(port, channel);
}


void RecordTool::startRecording(std::string path) {
    mStoragePath = path;
    mFrameCounter = 0;
    mRecording = true;

    createDirectories(mStoragePath);
    for (auto ch: mRecordChannels)
        createDirectories((mStoragePath + "/" + ch.first));

    update();
}

void RecordTool::stopRecording() {
    mRecording = false;
}


void RecordTool::execute()
{
    std::cout << "Enters" << std::endl;
    if (mRecording) {
        std::string frameNr = std::to_string(mFrameCounter);

        for (auto ch: mRecordChannels)
        {
            auto data = ch.second->getNextFrame();
            auto className = data->getNameOfClass();

            std::cout << className << std::endl;

            if(className == "Mesh")
            {
                VTKMeshFileExporter::pointer meshExporter = VTKMeshFileExporter::New();
                meshExporter->setInputData(data);
                meshExporter->setWriteNormals(false);
                meshExporter->setWriteColors(true);
                meshExporter->setFilename(mStoragePath + "/" + ch.first + "/" + frameNr  + ".vtk");
                meshExporter->update();
            }
            else if (className == "Image") {
                MetaImageExporter::pointer imageExporter = MetaImageExporter::New();
                imageExporter->setInputData(data);
                imageExporter->setFilename(mStoragePath + "/" + ch.first + "/" + "Image-2D_" + frameNr + ".mhd");
                imageExporter->update();
            }
        }
        ++mFrameCounter;
    }
}

RecordTool::RecordTool()
{
    mRecordThread = new std::thread(std::bind(&RecordTool::dataDumpThread, this));
}

void RecordTool::dataDumpThread()
{
}

}

