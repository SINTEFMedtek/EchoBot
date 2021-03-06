//
// Created by androst on 14.02.19.
//

#include <QLabel>
#include <QGridLayout>
#include <QApplication>

#include "ConnectionWidget.h"

namespace echobot
{

ConnectionWidget::ConnectionWidget(int widgetWidth, int widgetHeight) :
    mGraphicsFolderName("../../source/EchoBot/GUI/Widgets/Icons/"),
    mWidgetWidth(widgetWidth),
    mWidgetHeight(widgetHeight)
{
    this->setFixedWidth(mWidgetWidth);
    this->setFixedHeight(mWidgetHeight);
}

void ConnectionWidget::addInterface(SensorInterface::pointer sensorInterface)
{
    if(sensorInterface->getNameOfClass() == "RobotInterface"){
        mRobotInterface = std::dynamic_pointer_cast<RobotInterface>(sensorInterface);
        QWidget *robotConnectionWidget = createRobotConnectionWidget();
        this->addTab(robotConnectionWidget, "Robot");

        connect(mRobotConnectionButton, &QPushButton::clicked, this, &ConnectionWidget::robotToggleConnection);
        connect(mRobotShutdownButton, &QPushButton::clicked, this, &ConnectionWidget::robotShutdownSlot);

    } else if (sensorInterface->getNameOfClass() == "CameraInterface"){
        mCameraInterface = std::dynamic_pointer_cast<CameraInterface>(sensorInterface);
        QWidget *cameraConnectionWidget = createCameraConnectionWidget();
        this->addTab(cameraConnectionWidget, "Camera");

        connect(mCameraConnectionButton, &QPushButton::clicked, this, &ConnectionWidget::cameraToggleConnection);
        connect(mCameraMinDepthLineEdit, &QLineEdit::textChanged, this, &ConnectionWidget::updateCameraROI);
        connect(mCameraMaxDepthLineEdit, &QLineEdit::textChanged, this, &ConnectionWidget::updateCameraROI);
        connect(mCameraMinWidthLineEdit, &QLineEdit::textChanged, this, &ConnectionWidget::updateCameraROI);
        connect(mCameraMaxWidthLineEdit, &QLineEdit::textChanged, this, &ConnectionWidget::updateCameraROI);
        connect(mCameraMinHeightLineEdit, &QLineEdit::textChanged, this, &ConnectionWidget::updateCameraROI);
        connect(mCameraMaxHeightLineEdit, &QLineEdit::textChanged, this, &ConnectionWidget::updateCameraROI);
    } else if (sensorInterface->getNameOfClass() == "UltrasoundInterface")
    {
        mUltrasoundInterface = std::dynamic_pointer_cast<UltrasoundInterface>(sensorInterface);
        QWidget *usConnectionWidget = createUltrasoundConnectionWidget();
        this->addTab(usConnectionWidget, "Ultrasound");

        connect(mUSConnectionButton, &QPushButton::clicked, this, &ConnectionWidget::usToggleConnection);
        connect(mUSStreamerOptionCBox, &QComboBox::currentTextChanged, this, &ConnectionWidget::usStreamerChangedSlot);
    }
}


void ConnectionWidget::robotToggleConnection()
{
    mRobotConnected = !mRobotConnected;
    if(mRobotConnected){
        mRobotConnectionButton->setText("Disconnect");
        if(mRobotOptionCBox->currentText().toStdString() == "UR10")
        {
            auto manipulator = Manipulator(ManipulatorType::UR10, "5.3");
            mRobotInterface->setConfiguration(manipulator, mRobotIPLineEdit->text().toStdString(), 30003);
        }
        else if(mRobotOptionCBox->currentText().toStdString() == "UR5")
        {
            auto manipulator = Manipulator(ManipulatorType::UR5, "3.0");
            mRobotInterface->setConfiguration(manipulator, mRobotIPLineEdit->text().toStdString(), 30003);
        }
        mRobotInterface->connect();
        emit(this->robotConnected());
    }else{
        emit(this->robotDisconnected());
        mRobotInterface->disconnect();
        mRobotConnectionButton->setText("Connect");
    }
}


void ConnectionWidget::robotShutdownSlot()
{
    mRobotInterface->getRobot()->shutdown();
    emit(this->robotShutdown());
}

void ConnectionWidget::cameraToggleConnection()
{
    mCameraConnected = !mCameraConnected;
    if(mCameraConnected){
        mCameraConnectionButton->setText("Disconnect");
        mCameraInterface->connect();
        emit(this->cameraConnected());
    }else{
        emit(this->cameraDisconnected());
        mCameraInterface->disconnect();
        mCameraConnectionButton->setText("Connect");
    }
}

void ConnectionWidget::usToggleConnection()
{
    mUSConnected = !mUSConnected;
    if(mUSConnected){
        mUSConnectionButton->setText("Disconnect");
        if(mUSStreamerOptionCBox->currentText().toStdString() == "Clarius")
            mUltrasoundInterface->setStreamer(UltrasoundInterface::UltrasoundStreamerType::Clarius);
        else if(mUSStreamerOptionCBox->currentText().toStdString() == "IGTLink")
        {
            mUltrasoundInterface->setStreamer(UltrasoundInterface::UltrasoundStreamerType::IGTLink,
                                              mUsIPLineEdit->text().toStdString(), 18944);
        }
        mUltrasoundInterface->connect();
        emit(this->usConnected());

    } else {
        emit(this->usDisconnected());
        mUltrasoundInterface->disconnect();
        mUSConnectionButton->setText("Connect");
    }
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

QWidget* ConnectionWidget::createRobotConnectionWidget()
{
    QWidget *group = new QWidget;
    QGridLayout *mainLayout = new QGridLayout();
    group->setLayout(mainLayout);

    int row = 0;
    mRobotIPLineEdit = new QLineEdit();
    mRobotConnectionButton = new QPushButton();
    mainLayout->addWidget(new QLabel("IP Address: "), row, 0, 1, 1);
    mainLayout->addWidget(mRobotIPLineEdit, row, 1,1,1);
    mainLayout->addWidget(mRobotConnectionButton,row,2,1,1);

    mRobotIPLineEdit->setText("10.218.180.32"); // 10.218.140.123 10.218.140.114
    mRobotIPLineEdit->setAlignment(Qt::AlignCenter);

    QIcon icon;
    icon.addFile(mGraphicsFolderName+"network-idle.ico", QSize(), QIcon::Normal, QIcon::Off);
    icon.addFile(mGraphicsFolderName+"network-transmit-receive.ico", QSize(), QIcon::Normal, QIcon::On);
    mRobotConnectionButton->setIcon(icon);
    mRobotConnectionButton->setToolTip("Connect to robot");
    mRobotConnectionButton->setText("Connect");
    mRobotConnectionButton->setCheckable(true);
    mRobotConnectionButton->setStyleSheet("QPushButton:checked { background-color: white; }");
    mRobotConnectionButton->setFixedWidth(120);

    mRobotOptionCBox = new QComboBox();
    mRobotOptionCBox->setEditable(true);
    mRobotOptionCBox->lineEdit()->setReadOnly(true);
    mRobotOptionCBox->lineEdit()->setAlignment(Qt::AlignCenter);
    mRobotOptionCBox->addItem(tr("UR10"));
    mRobotOptionCBox->addItem(tr("UR5"));

    row++;
    mRobotShutdownButton = new QPushButton(QIcon(mGraphicsFolderName+"application-exit-4.png"),"Shutdown");
    mainLayout->addWidget(mRobotShutdownButton,row,0,1,1);
    mainLayout->addWidget(mRobotOptionCBox, row, 2, 1, 1);

    return group;
}

QWidget* ConnectionWidget::createUltrasoundConnectionWidget()
{
    QWidget *group = new QWidget;

    QGridLayout *mainLayout = new QGridLayout();
    group->setLayout(mainLayout);

    int row = 0;
    mUsIPLineEdit = new QLineEdit();
    mUSConnectionButton = new QPushButton();
    mUSStreamerOptionCBox = new QComboBox();

    mainLayout->addWidget(new QLabel("Streamer: "), row, 0, 1, 1);
    mainLayout->addWidget(mUSStreamerOptionCBox,row,1,1,1);
    mainLayout->addWidget(mUSConnectionButton,row,2,1,1);

    row++;
    mainLayout->addWidget(new QLabel("IP Address: "), row, 0, 1, 1);
    mainLayout->addWidget(mUsIPLineEdit, row, 1,1,1);

    mUSStreamerOptionCBox->setEditable(true);
    mUSStreamerOptionCBox->lineEdit()->setReadOnly(true);
    mUSStreamerOptionCBox->lineEdit()->setAlignment(Qt::AlignCenter);
    mUSStreamerOptionCBox->addItem(tr("IGTLink"));
    mUSStreamerOptionCBox->addItem(tr("Clarius"));

    mUsIPLineEdit->setText("localhost"); // 192.168.140.116, 10.218.140.114
    mUsIPLineEdit->setAlignment(Qt::AlignCenter);

    QIcon icon;
    icon.addFile(mGraphicsFolderName+"network-idle.ico", QSize(), QIcon::Normal, QIcon::Off);
    icon.addFile(mGraphicsFolderName+"network-transmit-receive.ico", QSize(), QIcon::Normal, QIcon::On);
    mUSConnectionButton->setIcon(icon);
    mUSConnectionButton->setToolTip("Connect to US Scanner");
    mUSConnectionButton->setText("Connect");
    mUSConnectionButton->setStyleSheet("QPushButton:checked { background-color: white; }");
    mUSConnectionButton->setFixedWidth(120);
    mUSConnectionButton->setCheckable(true);

    return group;
}

QWidget* ConnectionWidget::createCameraConnectionWidget()
{
    QWidget *group = new QWidget;

    QGridLayout *mainLayout = new QGridLayout();
    group->setLayout(mainLayout);

    mCameraMinDepthLineEdit = new QLineEdit();
    mCameraMaxDepthLineEdit = new QLineEdit();
    mCameraMinDepthLineEdit->setText(QString("1000")); // 0
    mCameraMaxDepthLineEdit->setText(QString("1700")); // 2000

    mCameraMinWidthLineEdit = new QLineEdit();
    mCameraMaxWidthLineEdit = new QLineEdit();
    mCameraMinWidthLineEdit->setText(QString("-400"));
    mCameraMaxWidthLineEdit->setText(QString("400"));

    mCameraMinHeightLineEdit = new QLineEdit();
    mCameraMaxHeightLineEdit = new QLineEdit();
    mCameraMinHeightLineEdit->setText(QString("-400"));
    mCameraMaxHeightLineEdit->setText(QString("400"));

    mainLayout->addWidget(new QLabel("Depth range [mm]: "), 0, 0, 1, 1);
    mainLayout->addWidget(mCameraMinDepthLineEdit,0,1,1,1);
    mainLayout->addWidget(mCameraMaxDepthLineEdit,0,2,1,1);

    mainLayout->addWidget(new QLabel("Width range [mm]: "), 1, 0, 1, 1);
    mainLayout->addWidget(mCameraMinWidthLineEdit,1,1,1,1);
    mainLayout->addWidget(mCameraMaxWidthLineEdit,1,2,1,1);

    mainLayout->addWidget(new QLabel("Height range [mm]: "), 2, 0, 1, 1);
    mainLayout->addWidget(mCameraMinHeightLineEdit,2,1,1,1);
    mainLayout->addWidget(mCameraMaxHeightLineEdit,2,2,1,1);

    mCameraConnectionButton = new QPushButton();

    QIcon icon;
    icon.addFile(mGraphicsFolderName+"network-idle.ico", QSize(), QIcon::Normal, QIcon::Off);
    icon.addFile(mGraphicsFolderName+"network-transmit-receive.ico", QSize(), QIcon::Normal, QIcon::On);
    mCameraConnectionButton->setIcon(icon);
    mCameraConnectionButton->setToolTip("Connect to robot");
    mCameraConnectionButton->setText("Connect");
    mCameraConnectionButton->setStyleSheet("QPushButton:checked { background-color: white; }");
    mCameraConnectionButton->setCheckable(true);
    mCameraConnectionButton->setFixedWidth(120);

    mainLayout->addWidget(mCameraConnectionButton,0,3,1,1);

    return group;
}

} // end namespace echobot