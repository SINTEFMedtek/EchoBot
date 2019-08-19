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

    createDirectories(mStoragePath);
    for (auto ch: mRecordChannels)
        createDirectories((mStoragePath + "/" + ch.first));

    mRecording = true;
    mRecordThread = new std::thread(std::bind(&RecordTool::dataDumpThread, this));
}

void RecordTool::stopRecording() {
    mRecording = false;
}


void RecordTool::execute()
{
}

RecordTool::RecordTool()
{
}

void RecordTool::dataDumpThread()
{
    while(true)
    {
        if(mRecording)
        {
            {
                std::lock_guard<std::mutex> lock(mRecordBufferMutex);
                if(!mRecording)
                    break;
            }

            std::string frameNr = std::to_string(mFrameCounter);

            for (auto ch: mRecordChannels) {
                mLatestData[ch.first] = ch.second->getNextFrame();
            }

            for(auto data: mLatestData)
            {
                auto className = data.second->getNameOfClass();

                if(className == "Mesh")
                {
                    VTKMeshFileExporter::pointer meshExporter = VTKMeshFileExporter::New();
                    meshExporter->setInputData(data.second);
                    meshExporter->setWriteNormals(false);
                    meshExporter->setWriteColors(true);
                    meshExporter->setFilename(mStoragePath + "/" + data.first + "/" + frameNr  + ".vtk");
                    meshExporter->update();
                }
                else if (className == "Image") {
                    MetaImageExporter::pointer imageExporter = MetaImageExporter::New();
                    imageExporter->setInputData(data.second);
                    imageExporter->setFilename(mStoragePath + "/" + data.first + "/" + "Image-2D_" + frameNr + ".mhd");
                    imageExporter->update();
                }
            }
            ++mFrameCounter;
            update();
        }
    }
}

uint RecordTool::getFramesStored() const {
    return mFrameCounter;
}

bool RecordTool::isRecording() const {
    return mRecording;
}


}

