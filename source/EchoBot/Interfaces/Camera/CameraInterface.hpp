#ifndef ECHOBOT_CAMERAINTERFACE_H
#define ECHOBOT_CAMERAINTERFACE_H

#include "EchoBot/Interfaces/SensorInterface.h"
#include "CameraDataProcessing.h"

#include "FAST/Visualization/Renderer.hpp"
#include "FAST/Visualization/ImageRenderer/ImageRenderer.hpp"
#include "FAST/Visualization/VertexRenderer/VertexRenderer.hpp"
#include "FAST/Streamers/RealSenseStreamer.hpp"

namespace echobot
{
using namespace fast;

class CameraInterface : public SensorInterface {
    ECHOBOT_OBJECT(CameraInterface)

    public:
        typedef enum {Stream, Playback} StreamOption;
        ~CameraInterface();

        void connect();
        void disconnect();
        bool isConnected(){return mConnected;};
        void setPlayback(std::string filepath);

        Renderer::pointer getPointCloudRenderer();
        Renderer::pointer getDepthImageRenderer();
        Renderer::pointer getImageRenderer();

        CameraDataProcessing::pointer getProcessObject(){ return mProcessObject;};
        DataChannel::pointer getOutputPort(uint portID = 0);
        Streamer::pointer getStreamObject(){ return mCameraStreamer;};

        void setCameraROI(  float minRange = 0, float maxRange = 2000,
                        float minWidth = -1000, float maxWidth = 1000,
                        float minHeight = -1000, float maxHeight = 1000);

    private:
        CameraInterface();

        SharedPointer<CameraDataProcessing> mProcessObject;
        SharedPointer<Streamer> mCameraStreamer;
        SharedPointer<VertexRenderer> mPointCloudRenderer;
        SharedPointer<ImageRenderer> mImageRenderer;
        SharedPointer<ImageRenderer> mDepthImageRenderer;
        StreamOption mStreamOption = StreamOption::Stream;
        std::string mPlaybackFilepath = "";
        bool mConnected = false;

};

}

#endif
