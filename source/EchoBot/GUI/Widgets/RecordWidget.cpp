//
// Created by androst on 14.02.19.
//

#include <QLabel>
#include <QGridLayout>
#include <QApplication>

#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QDir>
#include <QElapsedTimer>
#include <QListWidget>
#include <QDirIterator>
#include <QMessageBox>
#include <QGroupBox>
#include <QProgressDialog>
#include <QDir>
#include <QTabWidget>
#include <QFileDialog>

#include "RecordWidget.h"

RecordWidget::RecordWidget(SharedPointer<CameraInterface> cameraInterface, int widgetWidth) :
    mCameraInterface(cameraInterface),
    mWidgetWidth(widgetWidth)
{
    setupWidget();
    setupConnections();
}

void RecordWidget::setupWidget()
{
    QWidget *recordWidget = getRecordWidget();
    this->addTab(recordWidget, "Record exam");
}

void RecordWidget::setupConnections()
{

}

QWidget* RecordWidget::getRecordWidget()
{
    QGroupBox* group = new QGroupBox("Record exam");
    group->setFlat(true);

    QGridLayout *mainLayout = new QGridLayout();
    group->setLayout(mainLayout);

    mRecordTimer = new QElapsedTimer;

    QLabel* storageDirLabel = new QLabel;
    storageDirLabel->setText("Storage directory:");
    mainLayout->addWidget(storageDirLabel, 0, 0, 1, 1);

    mStorageDir = new QLineEdit;
    mStorageDir->setText(QDir::homePath() + QDir::separator() + QString("FAST_Kinect_Recordings"));
    mainLayout->addWidget(mStorageDir, 0, 1, 1, 1);

    QLabel* recordingNameLabel = new QLabel;
    recordingNameLabel->setText("Subject name:");
    mainLayout->addWidget(recordingNameLabel, 1, 0, 1, 1);

    mRecordingNameLineEdit = new QLineEdit;
    mainLayout->addWidget(mRecordingNameLineEdit, 1, 1, 1, 1);

    mRecordButton = new QPushButton;
    mRecordButton->setText("Record");
    mRecordButton->setStyleSheet("QPushButton { background-color: green; color: white; }");
    QObject::connect(mRecordButton, &QPushButton::clicked, std::bind(&RecordWidget::toggleRecord, this));
    mainLayout->addWidget(mRecordButton, 2, 0, 1, 1);

    mPlayButton = new QPushButton;
    mPlayButton->setText("Play");
    mPlayButton->setStyleSheet("QPushButton { background-color: green; color: white; }");
    mainLayout->addWidget(mPlayButton, 2, 1, 1, 1);
    QObject::connect(mPlayButton, &QPushButton::clicked, std::bind(&RecordWidget::playRecording, this));

    mRecordingInformation = new QLabel;
    mRecordingInformation->setStyleSheet("QLabel { font-size: 14px; }");
    mainLayout->addWidget(mRecordingInformation);

    mRecordingsList = new QListWidget;
    mainLayout->addWidget(mRecordingsList, 3, 0, 1, 2);
    mRecordingsList->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    mRecordingsList->setFixedHeight(40);
    mRecordingsList->setSortingEnabled(true);
    refreshRecordingsList();

    return group;
}

void RecordWidget::toggleRecord() {
    mRecording = !mRecording;
    if(mRecording) {
        mRecordButton->setText("Stop recording");
        mRecordButton->setStyleSheet("QPushButton { background-color: red; color: white; }");
        mStorageDir->setDisabled(true);
        mRecordTimer->start();

        // Create recording pathkom
        std::string path = mStorageDir->text().toUtf8().constData();
        if(mRecordingNameLineEdit->text() != "") {
            mRecordingName =  currentDateTime() + " " + mRecordingNameLineEdit->text().toUtf8().constData();
        } else {
            mRecordingName = currentDateTime();
        }
        std::string recordingPath = (QString(path.c_str()) + QDir::separator() + QString(mRecordingName.c_str()) + QDir::separator()).toUtf8().constData();
        createDirectories(recordingPath);

        std::cout << "Getting ready to start recording..." << std::endl;
        // Start saving point clouds
        mCameraInterface->startRecording(recordingPath);

    } else {
        mRecordButton->setText("Record");
        mRecordButton->setStyleSheet("QPushButton { background-color: green; color: white; }");
        mStorageDir->setDisabled(false);
        mCameraInterface->stopRecording();
        refreshRecordingsList();
    }
}

void RecordWidget::refreshRecordingsList() {
    // Get all folders in the folder mStorageDir
    QDirIterator it(mStorageDir->text());
    mRecordingsList->clear();
    while(it.hasNext()) {
        it.next();
        QString next = it.fileName();
        if(next.size() > 4)
            mRecordingsList->addItem(next);
    }
}

void RecordWidget::playRecording() {
    mCameraPlayback = !mCameraPlayback;
    if(!mCameraPlayback) {
        mPlayButton->setText("Play");
        mPlayButton->setStyleSheet("QPushButton { background-color: green; color: white; }");
        emit(playbackStopped());
    } else {
        auto selectedItems = mRecordingsList->selectedItems();
        if(selectedItems.size() == 0) {
            // Show error message
            QMessageBox *message = new QMessageBox;
            message->setWindowTitle("Error");
            message->setText("You did not select a recording.");
            message->show();
            return;
        }

        std::string selectedRecording = (
                mStorageDir->text() +
                QDir::separator() +
                selectedItems[0]->text() +
                QDir::separator()
        ).toUtf8().constData();

        std::string selectedRecordingPointClouds = selectedRecording + "/PointClouds/";
        std::string selectedRecordingImages = selectedRecording + "/CameraImages/";

        // Set up streaming from disk
        MeshFileStreamer::pointer meshStreamer = MeshFileStreamer::New();
        meshStreamer->setFilenameFormat(selectedRecordingPointClouds + "#.vtk");
        meshStreamer->enableLooping();
        meshStreamer->update(0);

        ImageFileStreamer::pointer imageStreamer = ImageFileStreamer::New();
        imageStreamer->setFilenameFormat(selectedRecordingImages + "Cam-2D_#.mhd");
        imageStreamer->enableLooping();
        imageStreamer->setSleepTime(100);
        imageStreamer->update(0);

        mCameraPlaybackStreamers[0] = imageStreamer;
        mCameraPlaybackStreamers[1] = meshStreamer;

        emit(this->playbackStarted(mCameraPlaybackStreamers));

        mPlayButton->setText("Stop");
        mPlayButton->setStyleSheet("QPushButton { background-color: red; color: white; }");
    }
}