#ifndef FAST_KINECT_TRACKING_HPP_
#define FAST_KINECT_TRACKING_HPP_

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
    private:
        CameraInterface();
        void execute();

        SharedPointer<Mesh> mCurrentCloud;
        bool mRecording = false;
        std::string mStoragePath;
        std::string mRecordingName;
        uint mFrameCounter;
};

}

#endif
