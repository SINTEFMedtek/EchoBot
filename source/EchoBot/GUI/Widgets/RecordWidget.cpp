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
#include <QCheckBox>
#include <QTimer>

#include "RecordWidget.h"

namespace echobot
{

RecordWidget::RecordWidget(SharedPointer<CameraInterface> cameraInterface, SharedPointer<UltrasoundInterface> usInterface,
                            int widgetWidth, int widgetHeight) :
    mCameraInterface(cameraInterface),
    mUltrasoundInterface(usInterface),
    mWidgetWidth(widgetWidth),
    mWidgetHeight(widgetHeight)
{
    setupWidget();
    setupConnections();

    this->setFixedWidth(mWidgetWidth);
    this->setFixedHeight(mWidgetHeight);

    mRecordTool = RecordTool::New();
}

void RecordWidget::setupWidget()
{
    QWidget *recordWidget = getRecordWidget();
    this->addTab(recordWidget, "Record exam");

    QWidget *settingsRecordWidget = getSettingsRecordWidget();
    this->addTab(settingsRecordWidget, "Settings");
}

void RecordWidget::setupConnections()
{
    QObject::connect(mPlayButton, &QPushButton::clicked, std::bind(&RecordWidget::playRecording, this));
    QObject::connect(mRecordButton, &QPushButton::clicked, std::bind(&RecordWidget::toggleRecord, this));
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
            mRecordingName =  fast::currentDateTime() + " " + mRecordingNameLineEdit->text().toUtf8().constData();
        } else {
            mRecordingName = fast::currentDateTime();
        }
        std::string recordingPath = (QString(path.c_str()) + QDir::separator() + QString(mRecordingName.c_str()) + QDir::separator()).toUtf8().constData();

        std::cout << "Getting ready to start recording..." << std::endl;
        if(mPointCloudDumpCheckBox->isChecked()  && mCameraInterface->isConnected())
            mRecordTool->addRecordChannel("PointClouds", mCameraInterface->getOutputPort(3));

        if(mImageDumpCheckBox->isChecked() && mCameraInterface->isConnected())
            mRecordTool->addRecordChannel("CameraImages", mCameraInterface->getOutputPort(0));

        if(mUltrasoundDumpCheckBox->isChecked() && mUltrasoundInterface->isConnected())
            mRecordTool->addRecordChannel("Ultrasound", mUltrasoundInterface->getOutputPort(0));

        mRecordTool->startRecording(recordingPath);

        QTimer *timer = new QTimer(this);
        QObject::connect(timer, &QTimer::timeout, std::bind(&RecordWidget::updateQueueSize, this));
        timer->start(100);
    } else {
        mRecordButton->setText("Record");
        mRecordButton->setStyleSheet("QPushButton { background-color: green; color: white; }");
        mStorageDir->setDisabled(false);
        mRecordTool->stopRecording();
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

void RecordWidget::updateQueueSize() {
    auto queueSize = mRecordTool->getQueueSize();
    mRecordingInformation->setText("Data dumping: " + QString::number(queueSize) + " objects left. "
                                                                                   "Don't close application.");

    if(queueSize == 0)
        mRecordingInformation->setHidden(true);
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
        std::string selectedRecordingUS = selectedRecording + "/Ultrasound/";

        // Set up streaming from disk
        if(QDir(QString::fromStdString(selectedRecordingImages)).exists() &&
        QDir(QString::fromStdString(selectedRecordingPointClouds)).exists())
        {
            mCameraInterface->setPlayback(selectedRecording);
            mCameraInterface->connect();
        }

        if(QDir(QString::fromStdString(selectedRecordingUS)).exists())
        {
            mUltrasoundInterface->setPlayback(selectedRecordingUS + "Image-2D_#.mhd");
            mUltrasoundInterface->connect();
        }

        emit(this->playbackStarted());

        mPlayButton->setText("Stop");
        mPlayButton->setStyleSheet("QPushButton { background-color: red; color: white; }");
    }
}

QWidget* RecordWidget::getRecordWidget()
{
    QGroupBox* group = new QGroupBox();
    group->setFlat(true);

    QGridLayout *mainLayout = new QGridLayout();
    group->setLayout(mainLayout);

    mRecordTimer = new QElapsedTimer;

    QLabel* storageDirLabel = new QLabel;
    storageDirLabel->setText("Storage directory:");
    mainLayout->addWidget(storageDirLabel, 0, 0, 1, 1);

    mStorageDir = new QLineEdit;
    mStorageDir->setText(QDir::homePath() + QDir::separator() + QString("EchoBot_Recordings"));
    mainLayout->addWidget(mStorageDir, 0, 1, 1, 1);

    QLabel* recordingNameLabel = new QLabel;
    recordingNameLabel->setText("Subject name:");
    mainLayout->addWidget(recordingNameLabel, 1, 0, 1, 1);

    mRecordingNameLineEdit = new QLineEdit;
    mainLayout->addWidget(mRecordingNameLineEdit, 1, 1, 1, 1);

    mRecordButton = new QPushButton;
    mRecordButton->setText("Record");
    mRecordButton->setStyleSheet("QPushButton { background-color: green; color: white; }");
    mainLayout->addWidget(mRecordButton, 2, 0, 1, 1);

    mPlayButton = new QPushButton;
    mPlayButton->setText("Play");
    mPlayButton->setStyleSheet("QPushButton { background-color: green; color: white; }");
    mainLayout->addWidget(mPlayButton, 2, 1, 1, 1);

    mRecordingsList = new QListWidget;
    mainLayout->addWidget(mRecordingsList, 3, 0, 1, 2);
    mRecordingsList->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    mRecordingsList->setSortingEnabled(true);
    refreshRecordingsList();

    mRecordingInformation = new QLabel;
    mRecordingInformation->setStyleSheet("QLabel { font-size: 14px; }");
    mainLayout->addWidget(mRecordingInformation, 4, 0, 1, 2);

    return group;
}

QWidget* RecordWidget::getSettingsRecordWidget()
{
    QGroupBox* group = new QGroupBox();
    group->setFlat(true);

    QGridLayout *mainLayout = new QGridLayout();
    group->setLayout(mainLayout);

    mImageDumpCheckBox = new QCheckBox("Camera Images");
    mImageDumpCheckBox->setLayoutDirection(Qt::RightToLeft);
    mImageDumpCheckBox->setChecked(true);

    mPointCloudDumpCheckBox = new QCheckBox("Camera point clouds");
    mPointCloudDumpCheckBox->setLayoutDirection(Qt::RightToLeft);
    mPointCloudDumpCheckBox->setChecked(true);

    mUltrasoundDumpCheckBox = new QCheckBox("Ultrasound images");
    mUltrasoundDumpCheckBox->setLayoutDirection(Qt::RightToLeft);
    mUltrasoundDumpCheckBox->setChecked(true);

    mainLayout->addWidget(mImageDumpCheckBox, 0, 0, 1, 1);
    mainLayout->addWidget(mPointCloudDumpCheckBox, 1, 0, 1, 1);
    mainLayout->addWidget(mUltrasoundDumpCheckBox, 2, 0, 1, 1);

    return group;
}

}