#ifndef FASTROMO_CAMERAINTERFACE_H
#define FASTROMO_CAMERAINTERFACE_H

#include "FAST/ProcessObject.hpp"

namespace fast {

class Image;
class Mesh;
class KinectStreamer;

class CameraInterface : public ProcessObject {
    FAST_OBJECT(CameraInterface)
    public:
        void restart();
        void startRecording(std::string path);
        void stopRecording();
        uint getFramesStored() const;
        bool isRecording() const;

        void calculateTargetCloud(SharedPointer<KinectStreamer> streamer);
        void removeTargetCloud();

        SharedPointer<Mesh> getTargetCloud();

        void addLine(Vector2i start, Vector2i end);
private:
        CameraInterface();
        void execute();

        SharedPointer<Image> mCurrentImage;
        SharedPointer<Mesh> mCurrentCloud;

        SharedPointer<Image> mAnnotationImage;
        SharedPointer<Mesh> mTargetCloud;

        bool mTargetCloudExtracted = false;

        bool mRecording = false;
        std::string mStoragePath;
        std::string mRecordingName;
        uint mFrameCounter;
};

}

#endif
