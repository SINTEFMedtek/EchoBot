#ifndef ECHOBOT_RECORDAPPLICATIONGUI_H
#define ECHOBOT_RECORDAPPLICATIONGUI_H

#include "EchoBot/Interfaces/Ultrasound/UltrasoundInterface.hpp"
#include "EchoBot/Interfaces/Camera/CameraInterface.hpp"
#include "EchoBot/GUI/Widgets/ConnectionWidget.h"
#include "EchoBot/GUI/Widgets/RecordWidget.h"

#include "FAST/Visualization/Window.hpp"
#include "FAST/Streamers/Streamer.hpp"

namespace echobot {

class RecordApplicationGUI : public Window {
    FAST_OBJECT(RecordApplicationGUI)

    private:
        RecordApplicationGUI();

        SharedPointer<CameraInterface> mCameraInterface;
        SharedPointer<UltrasoundInterface> mUltrasoundInterface;

        ConnectionWidget* mConnectionWidget;
        RecordWidget* mRecordWidget;

        void connectToCamera();
        void disconnectFromCamera();
        void stopStreaming();
        void connectToUltrasound();
        void disconnectFromUltrasound();
        void setupUI();

        bool mCameraPlayback = false;
        bool mCameraStreaming = false;
        bool mUltrasoundStreaming = false;

        void setupCameraVisualization();
        void setupUltrasoundVisualization();
        void setupConnections();

    private slots:
        void playbackButtonSlot();
        void stopPlaybackButtonSlot();
};

}

#endif
