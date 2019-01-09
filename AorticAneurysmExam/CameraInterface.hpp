#ifndef FASTROMO_CAMERAINTERFACE_H
#define FASTROMO_CAMERAINTERFACE_H

#include <FAST/Data/Mesh.hpp>
#include "FAST/ProcessObject.hpp"

namespace fast {

class Image;
class Mesh;
class RealSenseStreamer;

class CameraInterface : public ProcessObject {
    FAST_OBJECT(CameraInterface)
    public:
        void restart();
        void startRecording(std::string path);
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
        CameraInterface();
        void execute();

        SharedPointer<Image> mCurrentImage;
        SharedPointer<Mesh> mCurrentCloud;

        SharedPointer<Image> mAnnotationImage;
        SharedPointer<Mesh> mTargetCloud;

        bool mTargetCloudExtracted = false;
        bool mTargetRegistered = false;
        bool mTargetCloudPlaced = false;

        bool mRecording = false;
        std::string mStoragePath;
        std::string mRecordingName;
        uint mFrameCounter;
};

}

#endif
