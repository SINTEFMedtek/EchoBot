#ifndef ECHOBOT_RECORDAPPLICATIONGUI_H
#define ECHOBOT_RECORDAPPLICATIONGUI_H

#include "EchoBot/Interfaces/UltrasoundInterface.hpp"
#include "EchoBot/Interfaces/CameraInterface.hpp"
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

        std::unordered_map<uint, Streamer::pointer> mCameraPlaybackStreamers;

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

        void setupCameraVisualization(bool cameraPlayback = false);
        void setupUltrasoundVisualization();
        void setupConnections();

        void reinitializeViews();

    private slots:
        void playbackButtonSlot(std::unordered_map<uint, Streamer::pointer> streamers);
        void stopPlaybackButtonSlot();
};

}

#endif
