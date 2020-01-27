#include <QGridLayout>
#include <QGroupBox>
#include <QTimer>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QString>

#include "CalibrationWidget.h"


namespace echobot
{

CalibrationWidget::CalibrationWidget(int widgetWidth, int widgetHeight) :
    mWidgetWidth(widgetWidth),
    mWidgetHeight(widgetHeight)
{
    setupWidget();
    setupConnections();

    this->setFixedWidth(mWidgetWidth);
    this->setFixedHeight(mWidgetHeight);

    mCalibrationTool = CalibrationTool::New();
    this->updateSpinBoxes();
}

void CalibrationWidget::setupWidget()
{
    QWidget *recordWidget = getCalibrationWidget();
    this->addTab(recordWidget, "Calibration");
}

void CalibrationWidget::setupConnections()
{
    QObject::connect(mCalibrateButton, &QPushButton::clicked, std::bind(&CalibrationWidget::calibrateSystem, this));
    QObject::connect(mMatrixComboBox, SIGNAL(currentIndexChanged(QString)), this, SLOT(updateSpinBoxes()));
    QObject::connect(mXSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updateCalibration()));
    QObject::connect(mYSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updateCalibration()));
    QObject::connect(mZSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updateCalibration()));
    QObject::connect(mRXSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updateCalibration()));
    QObject::connect(mRYSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updateCalibration()));
    QObject::connect(mRZSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updateCalibration()));
    QObject::connect(mSaveMatrixButton, &QPushButton::clicked, std::bind(&CalibrationWidget::saveMatrixToFile, this));
}

void CalibrationWidget::updateSpinBoxes()
{
    auto vec = toVector6D(Eigen::Affine3d::Identity());
    if(mMatrixComboBox->currentIndex() == 0)
        vec = toVector6D(mCalibrationTool->get_rMb());
    else if(mMatrixComboBox->currentIndex() == 1)
        vec = toVector6D(mCalibrationTool->get_eeMt());
    else if(mMatrixComboBox->currentIndex() == 2)
        vec = toVector6D(mCalibrationTool->get_tMus());
    else if(mMatrixComboBox->currentIndex() == 3)
        vec = toVector6D(mCalibrationTool->get_registration_pcMdata());
    else if(mMatrixComboBox->currentIndex() == 4)
        vec = toVector6D(mCalibrationTool->get_registration_pcMt());

    mXSpinBox->setValue(vec(0));
    mYSpinBox->setValue(vec(1));
    mZSpinBox->setValue(vec(2));
    mRXSpinBox->setValue(vec(3)*180/M_PI);
    mRYSpinBox->setValue(vec(4)*180/M_PI);
    mRZSpinBox->setValue(vec(5)*180/M_PI);
}

void CalibrationWidget::updateCalibration()
{
    auto vec = getVectorFromSpinboxes();
    auto affine = toAffine3DFromVector6D(vec);

    if(mMatrixComboBox->currentIndex() == 0)
        mCalibrationTool->set_rMb(affine);
    else if(mMatrixComboBox->currentIndex() == 1)
        mCalibrationTool->set_eeMt(affine);
    else if(mMatrixComboBox->currentIndex() == 2)
        mCalibrationTool->set_tMus(affine);
    else if(mMatrixComboBox->currentIndex() == 3)
        mCalibrationTool->set_registration_pcMdata(affine);
    else if(mMatrixComboBox->currentIndex() == 4)
        mCalibrationTool->set_registration_pcMt(affine);
}

void CalibrationWidget::saveMatrixToFile() {
    auto vec = getVectorFromSpinboxes();
    auto affine = toAffine3DFromVector6D(vec);

    auto parent_path = mCalibrationTool->getCalibrationFilePath();

    if(mMatrixComboBox->currentIndex() == 0)
        mCalibrationTool->saveCalFile(parent_path+"camMbase.cal", affine);
    else if(mMatrixComboBox->currentIndex() == 1)
        mCalibrationTool->saveCalFile(parent_path+"eeMtool.cal", affine);
    else if(mMatrixComboBox->currentIndex() == 2)
        mCalibrationTool->saveCalFile(parent_path+"toolMus.cal", affine);
    else if(mMatrixComboBox->currentIndex() == 3)
        mCalibrationTool->saveCalFile(parent_path+"registration_pcMdata.cal", affine);
    else if(mMatrixComboBox->currentIndex() == 4)
        mCalibrationTool->saveCalFile(parent_path+"registration_pcMt.cal", affine);
}

QWidget* CalibrationWidget::getCalibrationWidget()
{
    QGroupBox* group = new QGroupBox();
    group->setFlat(true);

    auto mainLayout = new QVBoxLayout();

    mCalibrateButton = new QPushButton();
    mainLayout->addWidget(mCalibrateButton);
    mCalibrateButton->setText("Calibrate");
    mCalibrateButton->setStyleSheet("QPushButton:checked { background-color: none; }");

    auto calibrationModificationWidget = this->getCalibrationModificationWidget();
    mainLayout->addWidget(calibrationModificationWidget);

    group->setLayout(mainLayout);
    return group;
}

void CalibrationWidget::calibrateSystem() {
    mCalibrationTool->calibrate();
    mRobotInterface->getRobot()->getCoordinateSystem()->set_rMb(mCalibrationTool->get_rMb());
    mRobotInterface->getRobot()->getCoordinateSystem()->set_eeMt(mCalibrationTool->get_eeMt());

    if(mUltrasoundInterface->isConnected()){
        QTimer *timer = new QTimer(this);
        QObject::connect(timer, &QTimer::timeout, std::bind(&CalibrationWidget::updateToolToUSTransform, this));
        timer->start(25);
    }
}

void CalibrationWidget::addInterface(SensorInterface::pointer sensorInterface)
{
    if(sensorInterface->getNameOfClass() == "RobotInterface"){
        mRobotInterface = std::dynamic_pointer_cast<RobotInterface>(sensorInterface);
    } else if (sensorInterface->getNameOfClass() == "CameraInterface"){
        mCameraInterface = std::dynamic_pointer_cast<CameraInterface>(sensorInterface);
    } else if (sensorInterface->getNameOfClass() == "UltrasoundInterface"){
        mUltrasoundInterface = std::dynamic_pointer_cast<UltrasoundInterface>(sensorInterface);
    }
}

void CalibrationWidget::updateToolToUSTransform() {
    auto currentImage = mUltrasoundInterface->getProcessObject()->getProcessedImage();

    Eigen::Affine3d toolMus = mCalibrationTool->get_tMus();

    if(mUltrasoundInterface->getStreamerType() == UltrasoundInterface::Clarius ||
            mUltrasoundInterface->getStreamerType() == UltrasoundInterface::Playback)
    {
        toolMus(0, 3) = -(double)(currentImage->getWidth()*currentImage->getSpacing()(0)/2);
    }

    Eigen::Affine3d rMb = mRobotInterface->getRobot()->getCoordinateSystem()->get_rMb();
    Eigen::Affine3d eeMt = mRobotInterface->getRobot()->getCoordinateSystem()->get_eeMt();
    Eigen::Affine3d bMee = mRobotInterface->getCurrentState()->getTransformToJoint(6);
    Eigen::Affine3d transform = rMb*bMee*eeMt*toolMus;

    mUltrasoundInterface->setImageTransform(transform);
}

QWidget* CalibrationWidget::getCalibrationModificationWidget()
{
    QWidget *group = new QWidget;
    auto layout = new QGridLayout();
    layout->setSpacing(0);
    layout->setMargin(0);

    mRXSpinBox = new QDoubleSpinBox();
    mRXSpinBox->setRange(-360, 360);

    mRYSpinBox = new QDoubleSpinBox();
    mRYSpinBox->setRange(-360, 360);

    mRZSpinBox = new QDoubleSpinBox();
    mRZSpinBox->setRange(-360, 360);

    mXSpinBox = new QDoubleSpinBox();
    mXSpinBox->setRange(-3000, 3000);

    mYSpinBox = new QDoubleSpinBox();
    mYSpinBox->setRange(-3000, 3000);

    mZSpinBox = new QDoubleSpinBox();
    mZSpinBox->setRange(-3000, 3000);

    mMatrixComboBox = new QComboBox();
    mMatrixComboBox->setEditable(true);
    mMatrixComboBox->lineEdit()->setReadOnly(true);
    mMatrixComboBox->lineEdit()->setAlignment(Qt::AlignCenter);
    mMatrixComboBox->addItem(tr("Camera to robot base"), 0);
    mMatrixComboBox->addItem(tr("End-effector to tool"), 1);
    mMatrixComboBox->addItem(tr("Tool to ultrasound"), 2);
    mMatrixComboBox->addItem(tr("Registration: Point cloud to data"));
    mMatrixComboBox->addItem(tr("Registration: Point cloud to surface"));

    mSaveMatrixButton = new QPushButton("Save");

    int row = 0;
    layout->addWidget(new QLabel("Transform: "), row, 0, 1, 1);
    layout->addWidget(mMatrixComboBox,row, 1, 1, 3);
    layout->addWidget(mSaveMatrixButton, row, 5, 1, 1);

    row++;
    layout->addItem(new QSpacerItem(0, 10), row, 0, 1, 1);

    row++;
    layout->addWidget(new QLabel("X"), row, 0, 1, 1);
    layout->addWidget(mXSpinBox, row, 1, 1, 1);
    layout->addWidget(new QLabel(" mm"), row, 2, 1, 1);

    layout->addWidget(new QLabel("Rx"), row, 3, 1, 1, Qt::AlignHCenter);
    layout->addWidget(mRXSpinBox, row, 4, 1, 1);
    layout->addWidget(new QLabel(" Deg"), row, 5, 1, 1);

    row++;
    layout->addWidget(new QLabel("Y"), row, 0, 1, 1);
    layout->addWidget(mYSpinBox, row, 1, 1, 1);
    layout->addWidget(new QLabel(" mm"), row, 2, 1, 1);

    layout->addWidget(new QLabel("Ry"), row, 3, 1, 1, Qt::AlignHCenter);
    layout->addWidget(mRYSpinBox, row, 4, 1, 1);
    layout->addWidget(new QLabel(" Deg"), row, 5, 1, 1);

    row++;
    layout->addWidget(new QLabel("Z"), row, 0, 1, 1);
    layout->addWidget(mZSpinBox,row, 1, 1, 1);
    layout->addWidget(new QLabel(" mm"), row, 2, 1, 1);

    layout->addWidget(new QLabel("Rz"), row, 3, 1, 1, Qt::AlignHCenter);
    layout->addWidget(mRZSpinBox, row, 4, 1, 1);
    layout->addWidget(new QLabel(" Deg"), row, 5, 1, 1);

    group->setLayout(layout);
    return group;
}

Vector6d CalibrationWidget::getVectorFromSpinboxes() {
    auto vec = toVector6D(Eigen::Affine3d::Identity());
    vec(0) = mXSpinBox->value();
    vec(1) = mYSpinBox->value();
    vec(2) = mZSpinBox->value();
    vec(3) = mRXSpinBox->value()*M_PI/180.;
    vec(4) = mRYSpinBox->value()*M_PI/180.;
    vec(5) = mRZSpinBox->value()*M_PI/180.;
    return vec;
}

}