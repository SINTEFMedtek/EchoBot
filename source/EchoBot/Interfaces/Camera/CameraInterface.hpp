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
        ~CameraInterface();

        void connect();
        void disconnect();
        bool isConnected(){return mConnected;};

        Renderer::pointer getPointCloudRenderer();
        Renderer::pointer getDepthImageRenderer();
        Renderer::pointer getImageRenderer();

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
        SharedPointer<VertexRenderer> mPointCloudRenderer;
        SharedPointer<ImageRenderer> mImageRenderer;
        SharedPointer<ImageRenderer> mDepthImageRenderer;

        bool mConnected = false;

};

}

#endif
