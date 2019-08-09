#ifndef ECHOBOT_CAMERAINTERFACE_H
#define ECHOBOT_CAMERAINTERFACE_H

#include "EchoBot/Interfaces/SensorInterface.h"

#include "FAST/Data/Mesh.hpp"
#include "FAST/Streamers/Streamer.hpp"
#include "FAST/Streamers/RealSenseStreamer.hpp"
#include "FAST/ProcessObject.hpp"

namespace echobot
{
using namespace fast;

class CameraDataProcessing : public ProcessObject {
    ECHOBOT_OBJECT(CameraDataProcessing)

    public:
        void restart();
        void startRecording(std::string path, bool recordPointClouds = true, bool recordImages = true);
        void stopRecording();
        uint getFramesStored() const;
        bool isRecording() const;

        void calculateTargetCloud(SharedPointer<RealSenseStreamer> streamer);
        void removeTargetCloud();
        bool isTargetCloudExtracted(){return mTargetCloudExtracted;};

        SharedPointer<Mesh> getTargetCloud();

        void addLine(Vector2i start, Vector2i end);

        Mesh::pointer createReducedSample(SharedPointer<Mesh> pointCloud, double fractionOfPointsToKeep);

    private:
        CameraDataProcessing();
        void execute();

        SharedPointer<Image> mCurrentImage;
        SharedPointer<Mesh> mCurrentCloud;

        SharedPointer<Image> mAnnotationImage;
        SharedPointer<Mesh> mTargetCloud;

        bool mTargetCloudExtracted = false;
        bool mTargetRegistered = false;
        bool mTargetCloudPlaced = false;
        bool mStoreImages = false;
        bool mStorePointClouds = false;

        bool mRecording = false;
        std::string mStoragePath;
        std::string mRecordingName;
        uint mFrameCounter;
};

class CameraInterface : public SensorInterface {
    ECHOBOT_OBJECT(CameraInterface)

    public:
        ~CameraInterface();

        void connect();

        CameraDataProcessing::pointer getProcessObject(){ return mProcessObject;};
        DataChannel::pointer getOutputPort(uint portID = 0);
        RealSenseStreamer::pointer getStreamObject(){ return mCameraStreamer;};

        void setCameraROI(  float minRange = 0, float maxRange = 2000,
                        float minWidth = -1000, float maxWidth = 1000,
                        float minHeight = -1000, float maxHeight = 1000);

    private:
        CameraInterface();

        SharedPointer<CameraDataProcessing> mProcessObject;
        SharedPointer<RealSenseStreamer> mCameraStreamer;

};

}

#endif
