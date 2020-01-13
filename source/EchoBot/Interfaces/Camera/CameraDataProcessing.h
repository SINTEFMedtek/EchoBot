#ifndef ECHOBOT_CAMERADATAPROCESSING_H
#define ECHOBOT_CAMERADATAPROCESSING_H

#include "EchoBot/Core/SmartPointers.h"
#include "EchoBot/Core/DataTypes.h"

#include "FAST/ProcessObject.hpp"
#include "FAST/Data/Mesh.hpp"
#include "FAST/Streamers/RealSenseStreamer.hpp"

namespace echobot
{

class CameraDataProcessing : public ProcessObject {
    ECHOBOT_OBJECT(CameraDataProcessing)

    public:
        void calculateTargetCloud(SharedPointer<fast::RealSenseStreamer> streamer);

        void removeTargetCloud();

        bool isTargetCloudExtracted() { return mTargetCloudExtracted; };

        SharedPointer<Mesh> getTargetCloud();

        void addLine(Eigen::Vector2i start, Eigen::Vector2i end);

        void setMaxRange(float range);
        void setMinRange(float range);
        void setMaxWidth(float range);
        void setMinWidth(float range);
        void setMaxHeight(float range);
        void setMinHeight(float range);

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

        float mMaxRange = std::numeric_limits<float>::max();
        float mMinRange = 0;
        float mMaxWidth = std::numeric_limits<float>::max();
        float mMinWidth = -std::numeric_limits<float>::max();
        float mMaxHeight = std::numeric_limits<float>::max();
        float mMinHeight = -std::numeric_limits<float>::max();
};

}

#endif //ECHOBOT_CAMERADATAPROCESSING_H
