#ifndef ECHOBOT_RECORDTOOL_H
#define ECHOBOT_RECORDTOOL_H

#include "EchoBot/Core/SmartPointers.h"
#include "EchoBot/Interfaces/SensorInterface.h"
#include "FAST/ProcessObject.hpp"
#include "FAST/Streamers/Streamer.hpp"
#include "FAST/Semaphore.hpp"
#include <thread>
#include <queue>

class QElapsedTimer;

namespace echobot {
using namespace fast;


class RecordTool : public ProcessObject {
    ECHOBOT_OBJECT(RecordTool)

    public:
        void addRecordChannel(std::string name, DataChannel::pointer channel);
        void startRecording(std::string path);
        void stopRecording();

        uint getFramesStored() const;
        bool isRecording() const;

    private:
        RecordTool();
        void execute();


        void dataDumpThread();
        void queueForDataDump();

        std::map<std::string, DataChannel::pointer> mRecordChannels;
        typedef std::map<std::string, SharedPointer<DataObject>> DataContainer;

        std::queue<DataContainer> mDataQueue;
        std::map<std::string, uint64_t>  mLastTimeStamps;
        bool mTimeStampUpdated = true;

        bool mRecording = false;
        std::string mStoragePath;
        uint mFrameCounter;

        std::thread* mRecordThread;
        std::thread* mDumpThread;
        std::mutex mRecordBufferMutex;

        std::unique_ptr<LightweightSemaphore> m_fillCount;
        std::unique_ptr<LightweightSemaphore> m_emptyCount;
};

}


#endif //ECHOBOT_RECORDTOOL_H
