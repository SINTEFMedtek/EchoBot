#ifndef ECHOBOT_RECORDWIDGET_H
#define ECHOBOT_RECORDWIDGET_H

#include <QTabWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>

#include "FAST/Streamers/ImageFileStreamer.hpp"
#include "FAST/Streamers/MeshFileStreamer.hpp"

#include "EchoBot/Interfaces/RobotInterface.h"
#include "EchoBot/Interfaces/UltrasoundInterface.hpp"
#include "EchoBot/Interfaces/CameraInterface.hpp"

class QPushButton;
class QLabel;
class QTabWidget;
class QElapsedTimer;
class QListWidget;

class RecordWidget : public QTabWidget
{
    Q_OBJECT

    public:
        RecordWidget(SharedPointer<CameraInterface> cameraInterface, int widgetWidth=540);

    signals:
        void recordingStarted();
        void playbackStarted(const std::unordered_map<uint, Streamer::pointer> playbackStreamers);
        void playbackStopped();

    private:
        void setupWidget();
        void setupConnections();

        int mWidgetWidth;

        SharedPointer<RobotInterface> mRobotInterface;
        SharedPointer<CameraInterface> mCameraInterface;

        QPushButton *mRecordButton, *mPlayButton;
        QLineEdit* mStorageDir, *mRecordingNameLineEdit;
        QLabel* mRecordingInformation;
        QElapsedTimer* mRecordTimer;
        QListWidget* mRecordingsList;
        std::string mRecordingName;
        std::unordered_map<uint, Streamer::pointer> mCameraPlaybackStreamers;

        QWidget* getRecordWidget();

        void refreshRecordingsList();
        void toggleRecord();
        void playRecording();

        bool mRecording = false;
        bool mCameraPlayback = false;
        bool mCameraStreaming = false;
};

#endif //ECHOBOT_RECORDWIDGET_H
