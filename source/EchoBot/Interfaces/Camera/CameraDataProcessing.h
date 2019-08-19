#ifndef ECHOBOT_CAMERADATAPROCESSING_H
#define ECHOBOT_CAMERADATAPROCESSING_H

#include "EchoBot/Core/SmartPointers.h"

#include "FAST/ProcessObject.hpp"
#include "FAST/Data/Mesh.hpp"
#include "FAST/Streamers/RealSenseStreamer.hpp"

namespace echobot
{
using namespace fast;

class CameraDataProcessing : public ProcessObject {
    ECHOBOT_OBJECT(CameraDataProcessing)

    public:
        void calculateTargetCloud(SharedPointer<RealSenseStreamer> streamer);

        void removeTargetCloud();

        bool isTargetCloudExtracted() { return mTargetCloudExtracted; };

        SharedPointer<Mesh> getTargetCloud();

        void addLine(Vector2i start, Vector2i end);

        Mesh::pointer createReducedSample(SharedPointer<Mesh> pointCloud, double fractionOfPointsToKeep);

    private:
        CameraDataProcessing();

        void execute();

        SharedPointer<Image> mCurrentImage;
        SharedPointer<Image> mCurrentDepthImage;
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

}

#endif //ECHOBOT_CAMERADATAPROCESSING_H
