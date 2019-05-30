#ifndef ECHOBOT_CAMERAINTERFACE_H
#define ECHOBOT_CAMERAINTERFACE_H

#include "EchoBot/Core/SmartPointers.h"

#include "FAST/Data/Mesh.hpp"
#include "FAST/Streamers/Streamer.hpp"
#include "FAST/Streamers/RealSenseStreamer.hpp"
#include "FAST/ProcessObject.hpp"

namespace echobot
{
using namespace fast;

class CameraInterface : public ProcessObject {
    ECHOBOT_OBJECT(CameraInterface)
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

        void setCameraStreamer(SharedPointer<RealSenseStreamer> streamer);

        void setCameraROI(  float minRange = 0, float maxRange = 2000,
                            float minWidth = -1000, float maxWidth = 1000,
                            float minHeight = -1000, float maxHeight = 1000);

private:
        CameraInterface();
        void execute();

        SharedPointer<Image> mCurrentImage;
        SharedPointer<Mesh> mCurrentCloud;

        SharedPointer<Image> mAnnotationImage;
        SharedPointer<Mesh> mTargetCloud;

        SharedPointer<RealSenseStreamer> mCameraStreamer;

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

}

#endif
