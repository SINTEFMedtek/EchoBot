#ifndef FASTROMO_ULTRASOUNDINTERFACE_H
#define FASTROMO_ULTRASOUNDINTERFACE_H

#include "FAST/ProcessObject.hpp"
#include "RobotInterface.h"

namespace fast {

class Image;

class UltrasoundInterface : public ProcessObject {
    FAST_OBJECT(UltrasoundInterface)

    public:
        void setRobotInterface(RobotInterfacePtr robotInterface);

    private:
        UltrasoundInterface();
        void execute();

        SharedPointer<Image> mCurrentImage;

        RobotInterfacePtr mRobotInterface;
        void transformImageToProbeCenter();
};

}

#endif
