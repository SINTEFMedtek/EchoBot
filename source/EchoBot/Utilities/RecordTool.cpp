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
    m_emptyCount = std::make_unique<LightweightSemaphore>(1000);

    mRecording = true;
    mRecordThread = new std::thread(std::bind(&RecordTool::queueForDataDump, this));
    mDumpThread = new std::thread(std::bind(&RecordTool::dataDumpThread, this));
}

void RecordTool::stopRecording() {
    mRecording = false;
}

void RecordTool::startPlayback(std::string path)
{
    mRecording = false;

    std::string selectedRecordingPointClouds = path + "/PointClouds/";
    std::string selectedRecordingImages = path + "/CameraImages/";
    std::string selectedRecordingUS = path + "/Ultrasound/";

    if(QDir(QString::fromStdString(selectedRecordingPointClouds)).exists())
    {
        MeshFileStreamer::pointer meshStreamer = MeshFileStreamer::New();
        meshStreamer->setFilenameFormat(selectedRecordingPointClouds + "#.vtk");
        meshStreamer->enableLooping();
        meshStreamer->update();
        mPlaybackStreamers["PointCloud"] = meshStreamer;
    }

    if(QDir(QString::fromStdString(selectedRecordingImages)).exists())
    {
        ImageFileStreamer::pointer imageStreamer = ImageFileStreamer::New();
        imageStreamer->setFilenameFormat(selectedRecordingImages + "Image-2D_#.mhd");
        imageStreamer->enableLooping();
        imageStreamer->setSleepTime(100);
        imageStreamer->update();
        mPlaybackStreamers["CameraImage"] = imageStreamer;
    }

    if(QDir(QString::fromStdString(selectedRecordingUS)).exists())
    {
        ImageFileStreamer::pointer usImageStreamer = ImageFileStreamer::New();
        usImageStreamer->setFilenameFormat(selectedRecordingUS + "Image-2D_#.mhd");
        usImageStreamer->enableLooping();
        usImageStreamer->setSleepTime(100);
        usImageStreamer->update();
        mPlaybackStreamers["Ultrasound"] = usImageStreamer;
    }
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

                std::map<std::string, SharedPointer<DataObject>> latestData;
                std::map<std::string, uint64_t>  latestTimeStamps;

                for (auto ch: mRecordChannels) {
                    latestData[ch.first] = ch.second->getNextFrame();
                    latestTimeStamps[ch.first] = latestData[ch.first]->getCreationTimestamp();
                }

                mTimeStampUpdated = false;
                for (auto stamp: latestTimeStamps){
                    if(stamp.second > mLastTimeStamps[stamp.first]){
                        mTimeStampUpdated  = true;
                        mLastTimeStamps[stamp.first] = stamp.second;
                    }
                }

                if(mTimeStampUpdated)
                    mDataQueue.push(latestData);
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

        for(auto data: mDataQueue.front())
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
        mDataQueue.pop();
        m_emptyCount->signal();
    }
}

//void RecordTool::dataDumpThread()
//{
//    while(true)
//    {
//        if(mRecording)
//        {
//            {
//                std::lock_guard<std::mutex> lock(mRecordBufferMutex);
//                if(!mRecording)
//                    break;
//            }
//
//            std::map<std::string, SharedPointer<DataObject>> latestData;
//            std::string frameNr = std::to_string(mFrameCounter);
//
//            for (auto ch: mRecordChannels) {
//                latestData[ch.first] = ch.second->getNextFrame();
//            }
//
//            mDataQueue.push(latestData);
//
//            for(auto data: latestData)
//            {
//                auto className = data.second->getNameOfClass();
//
//                if(className == "Mesh")
//                {
//                    VTKMeshFileExporter::pointer meshExporter = VTKMeshFileExporter::New();
//                    meshExporter->setInputData(data.second);
//                    meshExporter->setWriteNormals(false);
//                    meshExporter->setWriteColors(true);
//                    meshExporter->setFilename(mStoragePath + "/" + data.first + "/" + frameNr  + ".vtk");
//                    meshExporter->update();
//                }
//                else if (className == "Image") {
//                    MetaImageExporter::pointer imageExporter = MetaImageExporter::New();
//                    imageExporter->setInputData(data.second);
//                    imageExporter->setFilename(mStoragePath + "/" + data.first + "/" + "Image-2D_" + frameNr + ".mhd");
//                    imageExporter->update();
//                }
//            }
//            ++mFrameCounter;
//            update();
//        }
//    }
//}


void RecordTool::addDataToQueue(std::map<std::string, SharedPointer<DataObject>> data)
{
    m_emptyCount->wait();

    {
        std::unique_lock<std::mutex> lock(mRecordBufferMutex);

        // If stop is signaled, throw an exception to stop the entire computation thread
        if(mStop)
            throw ThreadStopped();

        mDataQueue.push(data);
    }

    // Decrement semaphore by one, signal any waiting due to empty queue
    m_fillCount->signal();
}

RecordTool::DataContainer RecordTool::getNextDataFromQueue()
{

    RecordTool::DataContainer data;
    {
        std::unique_lock<std::mutex> lock(mRecordBufferMutex);

        // If stop is signaled, throw an exception to stop the entire computation thread
        if(mStop)
            throw ThreadStopped();

        // Get frame next in queue and remove it from the queue
        data = mDataQueue.front();
        mDataQueue.pop();
    }

    // Increment semaphore by one and signal any waiting for next frame due to empty queue
    m_emptyCount->signal();

    return data;
}



uint RecordTool::getFramesStored() const {
    return mFrameCounter;
}

bool RecordTool::isRecording() const {
    return mRecording;
}


}

