//
// Created by androst on 14.02.19.
//

#include <QLabel>
#include <QGridLayout>
#include <QApplication>

#include "ConnectionWidget.h"

namespace echobot
{

ConnectionWidget::ConnectionWidget(int widgetWidth) :
    mGraphicsFolderName("../icons/"),
    mWidgetWidth(widgetWidth)
{
    this->setFixedWidth(mWidgetWidth);
}

void ConnectionWidget::addInterface(SensorInterface::pointer sensorInterface)
{
    if(sensorInterface->getNameOfClass() == "RobotInterface"){
        mRobotInterface = std::dynamic_pointer_cast<RobotInterface>(sensorInterface);
        QWidget *robotConnectionWidget = getRobotConnectionWidget();
        this->addTab(robotConnectionWidget, "Robot");

        connect(mRobotConnectButton, &QPushButton::clicked, this, &ConnectionWidget::robotConnectSlot);
        connect(mRobotDisconnectButton, &QPushButton::clicked, this, &ConnectionWidget::robotDisconnectSlot);
        connect(mRobotShutdownButton, &QPushButton::clicked, this, &ConnectionWidget::robotShutdownSlot);

    } else if (sensorInterface->getNameOfClass() == "CameraInterface"){
        mCameraInterface = std::dynamic_pointer_cast<CameraInterface>(sensorInterface);
        QWidget *cameraConnectionWidget = getCameraConnectionWidget();
        this->addTab(cameraConnectionWidget, "Camera");

        connect(mCameraConnectButton, &QPushButton::clicked, this, &ConnectionWidget::cameraConnectSlot);
        connect(mCameraDisconnectButton, &QPushButton::clicked, this, &ConnectionWidget::cameraDisconnectSlot);
        connect(mCameraMinDepthLineEdit, &QLineEdit::textChanged, this, &ConnectionWidget::updateCameraROI);
        connect(mCameraMaxDepthLineEdit, &QLineEdit::textChanged, this, &ConnectionWidget::updateCameraROI);
        connect(mCameraMinWidthLineEdit, &QLineEdit::textChanged, this, &ConnectionWidget::updateCameraROI);
        connect(mCameraMaxWidthLineEdit, &QLineEdit::textChanged, this, &ConnectionWidget::updateCameraROI);
        connect(mCameraMinHeightLineEdit, &QLineEdit::textChanged, this, &ConnectionWidget::updateCameraROI);
        connect(mCameraMaxHeightLineEdit, &QLineEdit::textChanged, this, &ConnectionWidget::updateCameraROI);

    } else if (sensorInterface->getNameOfClass() == "UltrasoundInterface")
    {
        mUltrasoundInterface = std::dynamic_pointer_cast<UltrasoundInterface>(sensorInterface);
        QWidget *usConnectionWidget = getUltrasoundConnectionWidget();
        this->addTab(usConnectionWidget, "Ultrasound");

        connect(mUSConnectButton, &QPushButton::clicked, this, &ConnectionWidget::usConnectSlot);
        connect(mUSDisconnectButton, &QPushButton::clicked, this, &ConnectionWidget::usDisconnectSlot);
        connect(mUSStreamerOptionCBox, &QComboBox::currentTextChanged, this, &ConnectionWidget::usStreamerChangedSlot);
    }
}


void ConnectionWidget::robotConnectSlot()
{
    mRobotInterface->robot->configure(romocc::Manipulator::UR5, mRobotIPLineEdit->text().toStdString() ,30003);
    mRobotInterface->robot->start();

    if(mRobotInterface->robot->isConnected() && !mRobotConnectButton->isChecked())
    {
        mRobotConnectButton->toggle();
    }
    else if(!mRobotInterface->robot->isConnected() && mRobotConnectButton->isChecked())
    {
        mRobotConnectButton->toggle();
    }
    emit(this->robotConnected());
}

void ConnectionWidget::robotDisconnectSlot()
{
    mRobotInterface->robot->disconnectFromRobot();

    if(!mRobotInterface->robot->isConnected() && mRobotConnectButton->isChecked())
        mRobotConnectButton->toggle();

    emit(this->robotDisconnected());
}

void ConnectionWidget::robotShutdownSlot()
{
    mRobotInterface->robot->shutdown();
    emit(this->robotShutdown());
}

void ConnectionWidget::cameraConnectSlot()
{
    mCameraInterface->connect();
    emit(this->cameraConnected());
}

void ConnectionWidget::cameraDisconnectSlot()
{
    mCameraConnectButton->toggle();
    emit(this->cameraDisconnected());
}

void ConnectionWidget::usConnectSlot()
{
    if(mUSStreamerOptionCBox->currentText().toStdString() == "Clarius")
        mUltrasoundInterface->setStreamer(UltrasoundInterface::UltrasoundStreamer::Clarius);
    else if(mUSStreamerOptionCBox->currentText().toStdString() == "IGTLink")
    {
        mUltrasoundInterface->setStreamer(UltrasoundInterface::UltrasoundStreamer::IGTLink,
                mUsIPLineEdit->text().toStdString(), 18944);
    }
    mUltrasoundInterface->connect();
    emit(this->usConnected());
}

void ConnectionWidget::usDisconnectSlot()
{
    emit(this->usDisconnected());
}

void ConnectionWidget::usStreamerChangedSlot(const QString streamerName)
{
    // TODO: Remove IPLineEdit if Clarius etc
}

void ConnectionWidget::updateCameraROI(){
    mCameraInterface->setCameraROI(mCameraMinDepthLineEdit->text().toFloat(), mCameraMaxDepthLineEdit->text().toFloat(),
                                   mCameraMinWidthLineEdit->text().toFloat(), mCameraMaxWidthLineEdit->text().toFloat(),
                                   mCameraMinHeightLineEdit->text().toFloat(),mCameraMaxHeightLineEdit->text().toFloat());
}

QWidget* ConnectionWidget::getRobotConnectionWidget()
{
    QWidget *group = new QWidget;
    QGridLayout *mainLayout = new QGridLayout();
    group->setLayout(mainLayout);

    int row = 0;
    mRobotIPLineEdit = new QLineEdit();
    mRobotConnectButton = new QPushButton();
    mainLayout->addWidget(new QLabel("IP Address: "), row, 0, 1, 1);
    mainLayout->addWidget(mRobotIPLineEdit, row, 1,1,1);
    mainLayout->addWidget(mRobotConnectButton,row,2,1,1);

    mRobotIPLineEdit->setText("10.218.140.123"); // 10.218.140.114
    mRobotIPLineEdit->setAlignment(Qt::AlignCenter);

    QIcon icon;
    icon.addFile(mGraphicsFolderName+"network-idle.ico", QSize(), QIcon::Normal, QIcon::Off);
    icon.addFile(mGraphicsFolderName+"network-transmit-receive.ico", QSize(), QIcon::Normal, QIcon::On);
    mRobotConnectButton->setIcon(icon);
    mRobotConnectButton->setToolTip("Connect to robot");
    mRobotConnectButton->setText("Connect");
    mRobotConnectButton->setCheckable(true);
    mRobotConnectButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    row++;
    mRobotShutdownButton = new QPushButton(QIcon(mGraphicsFolderName+"application-exit-4.png"),"Shutdown");
    mRobotDisconnectButton = new QPushButton(QIcon(mGraphicsFolderName+"network-offline.ico"),"Disconnect");
    mainLayout->addWidget(mRobotShutdownButton,row,0,1,1);
    mainLayout->addWidget(mRobotDisconnectButton,row,2,1,1);

    return group;
}

QWidget* ConnectionWidget::getUltrasoundConnectionWidget()
{
    QWidget *group = new QWidget;

    QGridLayout *mainLayout = new QGridLayout();
    group->setLayout(mainLayout);

    int row = 0;
    mUsIPLineEdit = new QLineEdit();
    mUSConnectButton = new QPushButton();
    mUSStreamerOptionCBox = new QComboBox();

    mainLayout->addWidget(new QLabel("Streamer: "), row, 0, 1, 1);
    mainLayout->addWidget(mUSStreamerOptionCBox,row,1,1,1);
    mainLayout->addWidget(mUSConnectButton,row,2,1,1);

    row++;
    mainLayout->addWidget(new QLabel("IP Address: "), row, 0, 1, 1);
    mainLayout->addWidget(mUsIPLineEdit, row, 1,1,1);

    mUSStreamerOptionCBox->setEditable(true);
    mUSStreamerOptionCBox->lineEdit()->setReadOnly(true);
    mUSStreamerOptionCBox->lineEdit()->setAlignment(Qt::AlignCenter);
    mUSStreamerOptionCBox->addItem(tr("IGTLink"));
    mUSStreamerOptionCBox->addItem(tr("Clarius"));

    mUsIPLineEdit->setText("192.168.140.116"); // 10.218.140.114
    mUsIPLineEdit->setAlignment(Qt::AlignCenter);

    QIcon icon;
    icon.addFile(mGraphicsFolderName+"network-idle.ico", QSize(), QIcon::Normal, QIcon::Off);
    icon.addFile(mGraphicsFolderName+"network-transmit-receive.ico", QSize(), QIcon::Normal, QIcon::On);
    mUSConnectButton->setIcon(icon);
    mUSConnectButton->setToolTip("Connect to US Scanner");
    mUSConnectButton->setText("Connect");
    mUSConnectButton->setCheckable(true);
    mUSConnectButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    row++;
    mUSDisconnectButton = new QPushButton(QIcon(mGraphicsFolderName+"network-offline.ico"),"Disconnect");
    mainLayout->addWidget(mUSDisconnectButton,row,2,1,1);

    return group;
}

QWidget* ConnectionWidget::getCameraConnectionWidget()
{
    QWidget *group = new QWidget;

    QGridLayout *mainLayout = new QGridLayout();
    group->setLayout(mainLayout);

    mCameraMinDepthLineEdit = new QLineEdit();
    mCameraMaxDepthLineEdit = new QLineEdit();
    mCameraMinDepthLineEdit->setText(QString("0"));
    mCameraMaxDepthLineEdit->setText(QString("2000"));

    mCameraMinWidthLineEdit = new QLineEdit();
    mCameraMaxWidthLineEdit = new QLineEdit();
    mCameraMinWidthLineEdit->setText(QString("-1000"));
    mCameraMaxWidthLineEdit->setText(QString("1000"));

    mCameraMinHeightLineEdit = new QLineEdit();
    mCameraMaxHeightLineEdit = new QLineEdit();
    mCameraMinHeightLineEdit->setText(QString("-1000"));
    mCameraMaxHeightLineEdit->setText(QString("1000"));

    mainLayout->addWidget(new QLabel("Depth range [mm]: "), 0, 0, 1, 1);
    mainLayout->addWidget(mCameraMinDepthLineEdit,0,1,1,1);
    mainLayout->addWidget(mCameraMaxDepthLineEdit,0,2,1,1);

    mainLayout->addWidget(new QLabel("Width range [mm]: "), 1, 0, 1, 1);
    mainLayout->addWidget(mCameraMinWidthLineEdit,1,1,1,1);
    mainLayout->addWidget(mCameraMaxWidthLineEdit,1,2,1,1);

    mainLayout->addWidget(new QLabel("Height range [mm]: "), 2, 0, 1, 1);
    mainLayout->addWidget(mCameraMinHeightLineEdit,2,1,1,1);
    mainLayout->addWidget(mCameraMaxHeightLineEdit,2,2,1,1);

    mCameraConnectButton = new QPushButton();

    QIcon icon;
    icon.addFile(mGraphicsFolderName+"network-idle.ico", QSize(), QIcon::Normal, QIcon::Off);
    icon.addFile(mGraphicsFolderName+"network-transmit-receive.ico", QSize(), QIcon::Normal, QIcon::On);
    mCameraConnectButton->setIcon(icon);
    mCameraConnectButton->setToolTip("Connect to robot");
    mCameraConnectButton->setText("Connect");
    mCameraConnectButton->setCheckable(true);
    mCameraConnectButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    mainLayout->addWidget(mCameraConnectButton,0,3,1,1);

    mCameraDisconnectButton = new QPushButton(QIcon(mGraphicsFolderName+"network-offline.ico"),"Disconnect");
    mainLayout->addWidget(mCameraDisconnectButton,2,3,1,1);

    return group;
}

} // end namespace echobot