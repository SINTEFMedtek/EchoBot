#include "RecordTool.h"

#include <FAST/Exporters/VTKMeshFileExporter.hpp>
#include <FAST/Exporters/MetaImageExporter.hpp>
#include "FAST/Streamers/ImageFileStreamer.hpp"
#include "FAST/Streamers/MeshFileStreamer.hpp"
#include "EchoBot/Exporters/PointCloudExporter.h"

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

    mRecording = true;
    mRecordThread = new std::thread(std::bind(&RecordTool::queueForDataDump, this));

    if(mDumpThread == nullptr || mDataQueue.size() == 0)
        mDumpThread = new std::thread(std::bind(&RecordTool::dataDumpThread, this));
}

void RecordTool::stopRecording() {
    mRecording = false;
    mRecordThread->join();
}

void RecordTool::execute()
{
}

RecordTool::RecordTool()
{
    m_fillCount = std::make_unique<LightweightSemaphore>(0);
    m_emptyCount = std::make_unique<LightweightSemaphore>(4000);
}

void RecordTool::queueForDataDump()
{
    while(mRecording) {
        if(mTimeStampUpdated)
            m_emptyCount->wait();

        {
            std::unique_lock<std::mutex> lock(mRecordBufferMutex);
            if (!mRecording)
                break;

            std::map<std::string, DumpData> latestData;
            std::map<std::string, uint64_t>  latestTimeStamps;

            for (auto ch: mRecordChannels) {
                auto dumpData = DumpData(mStoragePath, mFrameCounter, ch.second->getNextFrame());
                latestData[ch.first] = dumpData;
                latestTimeStamps[ch.first] = latestData[ch.first].data->getCreationTimestamp();
            }

            mTimeStampUpdated = false;
            int count = 0;
            for (auto stamp: latestTimeStamps){
                if(stamp.second > mLastTimeStamps[stamp.first])
                    count = count+1;
            }

            if(count == latestTimeStamps.size())
            {
                mTimeStampUpdated = true;
                mLastTimeStamps = latestTimeStamps;
                mDataQueue.push(latestData);
                ++mFrameCounter;
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

        for (auto data: mDataQueue.front()) {
            auto className = data.second.data->getNameOfClass();
            auto parentPath = data.second.storagePath + "/" + data.first + "/";
            std::string frameNr = std::to_string(data.second.frameNr);

            if (className == "Mesh") {
                auto meshExporter = PointCloudExporter::New();
                meshExporter->setInputData(data.second.data);
                meshExporter->setWriteNormals(false);
                meshExporter->setWriteColors(true);
                meshExporter->setFilename(parentPath + frameNr + ".vtk");
                meshExporter->update();
            } else if (className == "Image") {
                MetaImageExporter::pointer imageExporter = MetaImageExporter::New();
                imageExporter->setInputData(data.second.data);
                imageExporter->setFilename(parentPath + "Image-2D_" + frameNr + ".mhd");
                imageExporter->update();
            }
        }
        mDataQueue.pop();
        m_emptyCount->signal();
    }
}


uint RecordTool::getQueueSize() const {
    return mDataQueue.size();
}

bool RecordTool::isRecording() const {
    return mRecording;
}


}

