//
// Created by androst on 20.12.19.
//

#ifndef ECHOBOT_POINTCLOUDEXPORTER_H
#define ECHOBOT_POINTCLOUDEXPORTER_H

#include "EchoBot/Core/SmartPointers.h"
#include "FAST/Exporters/FileExporter.hpp"

namespace echobot {
using namespace fast;

class PointCloudExporter : public FileExporter {
    ECHOBOT_OBJECT(PointCloudExporter)

    public:
        void setWriteNormals(bool writeNormals);
        void setWriteColors(bool writeColors);

    private:
        PointCloudExporter();
        void execute();

        bool mWriteNormals;
        bool mWriteColors;

        std::stringstream ss;
};

}



#endif //ECHOBOT_POINTCLOUDEXPORTER_H
