#ifndef ECHOBOT_RECORDTOOL_H
#define ECHOBOT_RECORDTOOL_H

#include "EchoBot/Core/SmartPointers.h"
#include "EchoBot/Interfaces/SensorInterface.h"
#include "FAST/ProcessObject.hpp"
#include <thread>

class QElapsedTimer;

namespace echobot {
using namespace fast;


class RecordTool : public ProcessObject {
    ECHOBOT_OBJECT(RecordTool)

    public:
        void addRecordChannel(std::string name, DataChannel::pointer channel);

        void startRecording(std::string path);
        void stopRecording();

private:
        RecordTool();
        void execute();
        void dataDumpThread();

        std::map<std::string, DataChannel::pointer> mRecordChannels;
        std::unordered_map<uint, SharedPointer<DataObject>> mLatestData;

        bool mRecording = false;
        std::string mStoragePath;
        uint mFrameCounter;
        QElapsedTimer* mRecordTimer;

        std::thread* mRecordThread;
        std::mutex mRecordBufferMutex;
};

}


#endif //ECHOBOT_RECORDTOOL_H
