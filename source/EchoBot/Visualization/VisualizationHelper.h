#ifndef ECHOBOT_VISUALIZATIONHELPER_H
#define ECHOBOT_VISUALIZATIONHELPER_H

#include "EchoBot/Core/SmartPointers.h"
#include "EchoBot/Core/DataTypes.h"

#include "FAST/Visualization/LineRenderer/LineRenderer.hpp"

namespace echobot {

class VisualizationHelper {
    ECHOBOT_OBJECT(VisualizationHelper)

    public:
        static LineRenderer::pointer createCoordinateFrameRenderer(Eigen::Affine3f transform, float axisLength = 500);

    private:
        std::weak_ptr<VisualizationHelper> mPtr;

};

}

#endif //ECHOBOT_VISUALIZATIONHELPER_H
