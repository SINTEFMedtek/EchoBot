#ifndef FAST_KINECT_TRACKING_GUI_HPP_
#define FAST_KINECT_TRACKING_GUI_HPP_

#include "FAST/Visualization/MultiViewWindow.hpp"

#include "widgets/robotManualMoveTab.h"
#include "RobotInterface.h"

class QPushButton;
class QLabel;

namespace fast {

class KinectStreamer;
class KinectTracking;

class ApplicationGUI : public MultiViewWindow {
    FAST_OBJECT(ApplicationGUI)

    private:
        ApplicationGUI();
        RobotInterfacePtr mRobotInterface;

        SharedPointer<KinectStreamer> mStreamer;
        SharedPointer<KinectTracking> mTracking;

        RobotManualMoveLayout* mMoveLayout;

        void toggleCameraConnection();
        void toggleRobotConnection();

        void setupUI();
};

}

#endif
