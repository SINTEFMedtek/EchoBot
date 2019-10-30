#include "RecordTool.h"

#include <FAST/Exporters/VTKMeshFileExporter.hpp>
#include <FAST/Exporters/MetaImageExporter.hpp>
#include "FAST/Streamers/ImageFileStreamer.hpp"
#include "FAST/Streamers/MeshFileStreamer.hpp"

#include <QString>
#include <QDir>

namespace echobot {

void RecordTool::addRecordChannel(std::string name, DataChannel::pointer channel)
{
    mRecordChannels[name] = channel;
    mLastTimeStamps[name] = 0;

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

    m_fillCount = std::make_unique<LightweightSemaphore>(0);
    m_emptyCount = std::make_unique<LightweightSemaphore>(4000);

    mRecording = true;
    mRecordThread = new std::thread(std::bind(&RecordTool::queueForDataDump, this));
    mDumpThread = new std::thread(std::bind(&RecordTool::dataDumpThread, this));
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

void RecordTool::queueForDataDump()
{
    while(true) {
        if (mRecording) {
            if(mTimeStampUpdated)
                m_emptyCount->wait();

            {
                std::unique_lock<std::mutex> lock(mRecordBufferMutex);
                if (!mRecording)
                    break;

                std::map<std::string, DataObject::pointer> latestData;
                std::map<std::string, uint64_t>  latestTimeStamps;

                for (auto ch: mRecordChannels) {
                    latestData[ch.first] = ch.second->getNextFrame();
                    latestTimeStamps[ch.first] = latestData[ch.first]->getCreationTimestamp();
                }

                mTimeStampUpdated = false;
                int count = 0;
                for (auto stamp: latestTimeStamps){
                    //std::cout << stamp.second << " " << mLastTimeStamps[stamp.first] << " " << int(stamp.second > mLastTimeStamps[stamp.first]) << std::endl;
                    if(stamp.second > mLastTimeStamps[stamp.first])
                        count = count+1;
                }

                if(count == latestTimeStamps.size())
                {
                    mTimeStampUpdated = true;
                    mLastTimeStamps = latestTimeStamps;
                    mDataQueue.push(latestData);
                }
            }

            if(mTimeStampUpdated)
                m_fillCount->signal();
        }
    }
}


void RecordTool::dataDumpThread() {
    while (true) {
        m_fillCount->wait();

        if (mDataQueue.empty() && mRecording)
            continue;
        else if (mDataQueue.empty() && !mRecording)
            break;

        std::cout << mDataQueue.size() << std::endl;
        std::string frameNr = std::to_string(mFrameCounter);

        for (auto data: mDataQueue.front()) {
            auto className = data.second->getNameOfClass();

            if (className == "Mesh") {
                VTKMeshFileExporter::pointer meshExporter = VTKMeshFileExporter::New();
                meshExporter->setInputData(data.second);
                meshExporter->setWriteNormals(false);
                meshExporter->setWriteColors(true);
                meshExporter->setFilename(mStoragePath + "/" + data.first + "/" + frameNr + ".vtk");
                meshExporter->update();
            } else if (className == "Image") {
                MetaImageExporter::pointer imageExporter = MetaImageExporter::New();
                imageExporter->setInputData(data.second);
                imageExporter->setFilename(mStoragePath + "/" + data.first + "/" + "Image-2D_" + frameNr + ".mhd");
                imageExporter->update();
            }
        }
        ++mFrameCounter;
        mDataQueue.pop();
        m_emptyCount->signal();
    }
}


uint RecordTool::getFramesStored() const {
    return mFrameCounter;
}

bool RecordTool::isRecording() const {
    return mRecording;
}


}

