#include "RobotManualMoveTab.h"

#include <QWidget>
#include <QGridLayout>
#include <QLabel>
#include <QApplication>
#include <QGroupBox>
#include <QList>

namespace echobot
{

RobotManualMoveLayout::RobotManualMoveLayout(RobotInterface::pointer robotInterface) :
    mRobotInterface(robotInterface)
{
    setupLayout();

    this->connectMovementButtons();
    this->connectJointButtons();

    this->updatePositions();
    QObject::connect(mRobotInterface->robot.get(), &corah::Robot::stateUpdated, std::bind(&RobotManualMoveLayout::updatePositions, this));
}

RobotManualMoveLayout::~RobotManualMoveLayout()
{
}


QLayout* RobotManualMoveLayout::getLayout()
{
    return this->mainLayout;
}


void RobotManualMoveLayout::setupLayout()
{
    tabWindow = new QWidget;

    mainLayout = new QHBoxLayout();

    QWidget *leftColumnWidgets = new QWidget();
    QVBoxLayout *leftColumnLayout = new QVBoxLayout(leftColumnWidgets);

    QWidget *rightColumnWidgets = new QWidget();
    QVBoxLayout *rightColumnLayout = new QVBoxLayout(rightColumnWidgets);

    setMoveToolLayout(leftColumnLayout);
    setJointMoveWidget(leftColumnLayout);
    setCoordInfoWidget(rightColumnLayout);
    setMoveSettingsWidget(rightColumnLayout);

    mainLayout->addWidget(leftColumnWidgets,0,Qt::AlignTop|Qt::AlignLeft);
    mainLayout->addWidget(rightColumnWidgets,0,Qt::AlignTop|Qt::AlignLeft);

    tabWindow->setLayout(mainLayout);
}


void RobotManualMoveLayout::setMoveToolLayout(QVBoxLayout *parent)
{
    QGroupBox* group = new QGroupBox("Move Tool");
    group->setFont(QFont("Arial",8));
    group->setFlat(true);
    parent->addWidget(group);

    QGridLayout *keyLayout = new QGridLayout();
    group->setLayout(keyLayout);
    keyLayout->setSpacing(0);
    keyLayout->setMargin(0);
    keyLayout->setContentsMargins(0,0,0,0);

    posZButton = new QPushButton("+z"); //QIcon("/icons/arrow-up-double.png"),"");
    negZButton = new QPushButton("-z"); //QIcon("/icons/arrow-down-double.png"),"");
    posXButton = new QPushButton("+x"); //QIcon("/icons/arrow-up.png"),"");
    negXButton = new QPushButton("-x"); //QIcon("/icons/arrow-down.png"),"");
    posYButton = new QPushButton("+y"); //QIcon("/icons/arrow-left.png"),"");
    negYButton = new QPushButton("-y"); // QIcon("/icons/arrow-right.png"),"");

    linearMotionButtons = new QButtonGroup();

    linearMotionButtons->addButton(posZButton);
    linearMotionButtons->addButton(negZButton);
    linearMotionButtons->addButton(posXButton);
    linearMotionButtons->addButton(negXButton);
    linearMotionButtons->addButton(posYButton);
    linearMotionButtons->addButton(negYButton);

    this->setAutoRepeat(true,linearMotionButtons);
    this->setMaximumWidth(46,linearMotionButtons);

    posZButton->setToolTip("Move in positive Z direction");
    negZButton->setToolTip("Move in negative Z direction");
    posXButton->setToolTip("Move in positive Y direction");
    negXButton->setToolTip("Move in negative Y direction");
    posYButton->setToolTip("Move in positive X direction");
    negYButton->setToolTip("Move in negative X direction");

    rotPosXButton = new QPushButton("+rx");
    rotNegXButton = new QPushButton("-rx");
    rotPosYButton = new QPushButton("+ry");
    rotNegYButton = new QPushButton("-ry");
    rotPosZButton = new QPushButton("+rz");
    rotNegZButton = new QPushButton("-rz");

    rotationMotionButtons = new QButtonGroup();

    rotationMotionButtons->addButton(rotPosXButton);
    rotationMotionButtons->addButton(rotNegXButton);
    rotationMotionButtons->addButton(rotPosYButton);
    rotationMotionButtons->addButton(rotNegYButton);
    rotationMotionButtons->addButton(rotPosZButton);
    rotationMotionButtons->addButton(rotNegZButton);

    this->setAutoRepeat(true, rotationMotionButtons);
    this->setMaximumWidth(46, rotationMotionButtons);

    rotPosXButton->setToolTip("Rotate counter-clockwise around X axis");
    rotNegXButton->setToolTip("Rotate clockwise around X axis");
    rotPosYButton->setToolTip("Rotate counter-clockwise around Y axis");
    rotNegYButton->setToolTip("Rotate clockwise around Y axis");
    rotPosZButton->setToolTip("Rotate counter-clockwise around Z axis");
    rotNegZButton->setToolTip("Rotate clockwise around Z axis");

    int krow=0;
    keyLayout->addWidget(posZButton, krow, 0, 1, 1,Qt::AlignBottom);
    keyLayout->addWidget(negZButton, krow,2,1,1,Qt::AlignBottom);

    krow++;
    keyLayout->addWidget(posXButton,krow,1,1,1,Qt::AlignBottom);

    krow++;
    keyLayout->addWidget(posYButton,krow,0,1,1,Qt::AlignRight);
    keyLayout->addWidget(negYButton,krow,2,1,1,Qt::AlignLeft);

    krow++;
    keyLayout->addWidget(negXButton,krow,1,1,1,Qt::AlignTop);

    krow++;
    keyLayout->addWidget(rotNegXButton, krow, 0, 1, 1,Qt::AlignBottom);
    keyLayout->addWidget(rotPosXButton, krow,2,1,1,Qt::AlignBottom);

    krow++;
    keyLayout->addWidget(rotPosYButton,krow,1,1,1,Qt::AlignBottom);

    krow++;
    keyLayout->addWidget(rotNegZButton,krow,0,1,1,Qt::AlignRight);
    keyLayout->addWidget(rotPosZButton,krow,2,1,1,Qt::AlignLeft);

    krow++;
    keyLayout->addWidget(rotNegYButton,krow,1,1,1,Qt::AlignTop);
}

void RobotManualMoveLayout::setMoveSettingsWidget(QVBoxLayout *parent)
{
    QGroupBox* group = new QGroupBox("Movement Settings");
    group->setFont(QFont("Arial",8));
    group->setFlat(true);
    parent->addWidget(group);

    QGridLayout *velAccLayout = new QGridLayout();
    group->setLayout(velAccLayout);

    velAccLayout->setSpacing(5);
    velAccLayout->setMargin(5);

    // Velocity
    velAccLayout->addWidget(new QLabel("Vel"), 0, 0, 1, 1);
    velocityLineEdit = new QLineEdit();
    velAccLayout->addWidget(velocityLineEdit, 0, 1, 1, 1);
    velocityLineEdit->setText(QApplication::translate("Ur5Widget", "50", 0));
    velAccLayout->addWidget(new QLabel("mm/s"), 0, 2, 1, 1);

    // Acceleration
    accelerationLineEdit = new QLineEdit();
    velAccLayout->addWidget(accelerationLineEdit, 1, 1, 1, 1);
    accelerationLineEdit->setText(QApplication::translate("Ur5Widget", "250", 0));
    velAccLayout->addWidget(new QLabel("Acc"), 1, 0, 1, 1);
    velAccLayout->addWidget(new QLabel("mm/s^2"), 1, 2, 1, 1);

    // Time
    velAccLayout->addWidget(new QLabel("Time"), 2, 0, 1, 1);
    timeLineEdit = new QLineEdit();
    velAccLayout->addWidget(timeLineEdit, 2, 1, 1, 1);
    timeLineEdit->setText(QApplication::translate("Ur5Widget", "0.5", 0));
    velAccLayout->addWidget(new QLabel("s"), 2, 2, 1, 1);

    timeLineEdit->setMinimumWidth(40);
    velocityLineEdit->setMinimumWidth(40);
    accelerationLineEdit->setMinimumWidth(40);
}

void RobotManualMoveLayout::setCoordInfoWidget(QVBoxLayout *parent)
{
    QGroupBox* group = new QGroupBox("Tool Position");
    group->setFont(QFont("Arial",8));
    group->setFlat(true);
    parent->addWidget(group);

    QGridLayout *coordInfoLayout = new QGridLayout();
    group->setLayout(coordInfoLayout);

    coordInfoLayout->setSpacing(5);
    coordInfoLayout->setMargin(5);

    // Position label
    coordInfoLayout->addWidget(new QLabel("X"), 0, 0, 1, 1);
    coordInfoLayout->addWidget(new QLabel("Y"), 1, 0, 1, 1);
    coordInfoLayout->addWidget(new QLabel("Z"), 3, 0, 1, 1);

    // mm label
    coordInfoLayout->addWidget(new QLabel("mm"), 0, 3, 1, 1);
    coordInfoLayout->addWidget(new QLabel("mm"), 1, 3, 1, 1);
    coordInfoLayout->addWidget(new QLabel("mm"), 3, 3, 1, 1);

    // Ri orientation label
    coordInfoLayout->addWidget(new QLabel("RX"), 5, 0, 1, 1);
    coordInfoLayout->addWidget(new QLabel("RZ"), 7, 0, 1, 1);
    coordInfoLayout->addWidget(new QLabel("RY"), 6, 0, 1, 1);

    // Rad label
    coordInfoLayout->addWidget(new QLabel("Rad"), 5, 3, 1, 1);
    coordInfoLayout->addWidget(new QLabel("Rad"), 6, 3, 1, 1);
    coordInfoLayout->addWidget(new QLabel("Rad"), 7, 3, 1, 1);

    // X coordinate line edit
    xPosLineEdit = new QLineEdit();
    coordInfoLayout->addWidget(xPosLineEdit, 0, 2, 1, 1);

    // Y coordinate line edit
    yPosLineEdit = new QLineEdit();
    coordInfoLayout->addWidget(yPosLineEdit, 1, 2, 1, 1);

    // Line edit for Z position
    zPosLineEdit = new QLineEdit();
    coordInfoLayout->addWidget(zPosLineEdit, 3, 2, 1, 1);

    // Line edit for RX orientation
    rxLineEdit = new QLineEdit();
    coordInfoLayout->addWidget(rxLineEdit, 5, 2, 1, 1);

    // Line edit for RY orientation
    ryLineEdit = new QLineEdit();
    coordInfoLayout->addWidget(ryLineEdit, 6, 2, 1, 1);

    // Line edit for RZ orientation
    rzLineEdit = new QLineEdit();
    coordInfoLayout->addWidget(rzLineEdit, 7, 2, 1, 1);

}

void RobotManualMoveLayout::setJointMoveWidget(QVBoxLayout *parent)
{
    QGroupBox* group = new QGroupBox("Move Joints");
    group->setFont(QFont("Arial",8));
    group->setFlat(true);
    parent->addWidget(group);

    QGridLayout *coordInfoLayout = new QGridLayout();
    group->setLayout(coordInfoLayout);

    coordInfoLayout->setSpacing(5);
    coordInfoLayout->setMargin(5);

    q1PosButton = new QPushButton("+");
    q1NegButton = new QPushButton("-");
    q2PosButton = new QPushButton("+");
    q2NegButton = new QPushButton("-");
    q3PosButton = new QPushButton("+");
    q3NegButton = new QPushButton("-");
    q4PosButton = new QPushButton("+");
    q4NegButton = new QPushButton("-");
    q5PosButton = new QPushButton("+");
    q5NegButton = new QPushButton("-");
    q6PosButton = new QPushButton("+");
    q6NegButton = new QPushButton("-");

    jointConfigurationButtons = new QButtonGroup();

    jointConfigurationButtons->addButton(q1PosButton);
    jointConfigurationButtons->addButton(q2PosButton);
    jointConfigurationButtons->addButton(q3PosButton);
    jointConfigurationButtons->addButton(q4PosButton);
    jointConfigurationButtons->addButton(q5PosButton);
    jointConfigurationButtons->addButton(q6PosButton);
    jointConfigurationButtons->addButton(q1NegButton);
    jointConfigurationButtons->addButton(q2NegButton);
    jointConfigurationButtons->addButton(q3NegButton);
    jointConfigurationButtons->addButton(q4NegButton);
    jointConfigurationButtons->addButton(q5NegButton);
    jointConfigurationButtons->addButton(q6NegButton);

    this->setAutoRepeat(true, jointConfigurationButtons);
    this->setMaximumWidth(32, jointConfigurationButtons);

    q1PosButton->setToolTip("Move joint 1 in positive direction");
    q2PosButton->setToolTip("Move joint 2 in positive direction");
    q3PosButton->setToolTip("Move joint 3 in positive direction");
    q4PosButton->setToolTip("Move joint 4 in positive direction");
    q5PosButton->setToolTip("Move joint 5 in positive direction");
    q6PosButton->setToolTip("Move joint 6 in positive direction");
    q1NegButton->setToolTip("Move joint 1 in negative direction");
    q2NegButton->setToolTip("Move joint 2 in negative direction");
    q3NegButton->setToolTip("Move joint 3 in negative direction");
    q4NegButton->setToolTip("Move joint 4 in negative direction");
    q5NegButton->setToolTip("Move joint 5 in negative direction");
    q6NegButton->setToolTip("Move joint 6 in negative direction");

    q1LineEdit = new QLineEdit();
    q2LineEdit = new QLineEdit();
    q3LineEdit = new QLineEdit();
    q4LineEdit = new QLineEdit();
    q5LineEdit = new QLineEdit();
    q6LineEdit = new QLineEdit();

    int row = 0;
    coordInfoLayout->addWidget(new QLabel("Base"), row, 0, 1, 1, Qt::AlignHCenter);
    coordInfoLayout->addWidget(q1NegButton,row,1,1,1);
    coordInfoLayout->addWidget(q1PosButton,row,2,1,1);
    coordInfoLayout->addWidget(q1LineEdit, row, 3, 1, 1);
    coordInfoLayout->addWidget(new QLabel("Rad"), row, 4, 1, 1);

    row++;
    coordInfoLayout->addWidget(new QLabel("Shoulder"), row, 0, 1, 1, Qt::AlignHCenter);
    coordInfoLayout->addWidget(q2NegButton,row,1,1,1);
    coordInfoLayout->addWidget(q2PosButton,row,2,1,1);
    coordInfoLayout->addWidget(q2LineEdit, row, 3, 1, 1);
    coordInfoLayout->addWidget(new QLabel("Rad"), row, 4, 1, 1);

    row++;
    coordInfoLayout->addWidget(new QLabel("Elbow"), row, 0, 1, 1, Qt::AlignHCenter);
    coordInfoLayout->addWidget(q3NegButton,row,1,1,1);
    coordInfoLayout->addWidget(q3PosButton,row,2,1,1);
    coordInfoLayout->addWidget(q3LineEdit, row, 3, 1, 1);
    coordInfoLayout->addWidget(new QLabel("Rad"), row, 4, 1, 1);

    row++;
    coordInfoLayout->addWidget(new QLabel("Wrist 1"), row, 0, 1, 1);
    coordInfoLayout->addWidget(q4NegButton,row,1,1,1);
    coordInfoLayout->addWidget(q4PosButton,row,2,1,1);
    coordInfoLayout->addWidget(q4LineEdit, row, 3, 1, 1);
    coordInfoLayout->addWidget(new QLabel("Rad"), row, 4, 1, 1);

    row++;
    coordInfoLayout->addWidget(new QLabel("Wrist 2"), row, 0, 1, 1);
    coordInfoLayout->addWidget(q5NegButton,row,1,1,1);
    coordInfoLayout->addWidget(q5PosButton,row,2,1,1);
    coordInfoLayout->addWidget(q5LineEdit, row, 3, 1, 1);
    coordInfoLayout->addWidget(new QLabel("Rad"), row, 4, 1, 1);

    row++;
    coordInfoLayout->addWidget(new QLabel("Wrist 3"), row, 0, 1, 1);
    coordInfoLayout->addWidget(q6NegButton,row,1,1,1);
    coordInfoLayout->addWidget(q6PosButton,row,2,1,1);
    coordInfoLayout->addWidget(q6LineEdit, row, 3, 1, 1);
    coordInfoLayout->addWidget(new QLabel("Rad"), row, 4, 1, 1);
}

void RobotManualMoveLayout::setAutoRepeat(bool isRepeated, QButtonGroup *buttons)
{
    for(int i=0; i<buttons->buttons().size(); i++)
    {
        buttons->buttons()[i]->setAutoRepeat(isRepeated);
    }
}

void RobotManualMoveLayout::setMaximumWidth(int width, QButtonGroup *buttons)
{
    for(int i=0; i<buttons->buttons().size(); i++)
    {
        buttons->buttons()[i]->setMaximumWidth(width);
    }
}

void RobotManualMoveLayout::updatePositions()
{
    xPosLineEdit->setText(QString::number(mRobotInterface->robot->getCurrentState().operationalConfiguration(0),'f',2));
    yPosLineEdit->setText(QString::number(mRobotInterface->robot->getCurrentState().operationalConfiguration(1),'f',2));
    zPosLineEdit->setText(QString::number(mRobotInterface->robot->getCurrentState().operationalConfiguration(2),'f',2));
    rxLineEdit->setText(QString::number(mRobotInterface->robot->getCurrentState().operationalConfiguration(3),'f',4));
    ryLineEdit->setText(QString::number(mRobotInterface->robot->getCurrentState().operationalConfiguration(4),'f',4));
    rzLineEdit->setText(QString::number(mRobotInterface->robot->getCurrentState().operationalConfiguration(5),'f',4));
    q1LineEdit->setText(QString::number(mRobotInterface->robot->getCurrentState().jointConfiguration(0),'f',4));
    q2LineEdit->setText(QString::number(mRobotInterface->robot->getCurrentState().jointConfiguration(1),'f',4));
    q3LineEdit->setText(QString::number(mRobotInterface->robot->getCurrentState().jointConfiguration(2),'f',4));
    q4LineEdit->setText(QString::number(mRobotInterface->robot->getCurrentState().jointConfiguration(3),'f',4));
    q5LineEdit->setText(QString::number(mRobotInterface->robot->getCurrentState().jointConfiguration(4),'f',4));
    q6LineEdit->setText(QString::number(mRobotInterface->robot->getCurrentState().jointConfiguration(5),'f',4));
}


void RobotManualMoveLayout::connectMovementButtons()
{
    QObject::connect(posXButton,&QPushButton::pressed,  std::bind(&RobotManualMoveLayout::posXButtonPressed, this));
    QObject::connect(posXButton,&QPushButton::released, std::bind(&RobotManualMoveLayout::moveButtonReleased, this));

    QObject::connect(negXButton,&QPushButton::pressed, std::bind(&RobotManualMoveLayout::negXButtonPressed, this));
    QObject::connect(negXButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::moveButtonReleased, this));

    QObject::connect(posYButton,&QPushButton::pressed, std::bind(&RobotManualMoveLayout::posYButtonPressed, this));
    QObject::connect(posYButton,&QPushButton::released, std::bind(&RobotManualMoveLayout::moveButtonReleased, this));

    QObject::connect(negYButton,&QPushButton::pressed, std::bind(&RobotManualMoveLayout::negYButtonPressed, this));
    QObject::connect(negYButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::moveButtonReleased, this));

    QObject::connect(posZButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::posZButtonPressed, this));
    QObject::connect(posZButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::moveButtonReleased, this));

    QObject::connect(negZButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::negZButtonPressed, this));
    QObject::connect(negZButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::moveButtonReleased, this));

    QObject::connect(rotPosXButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::posRXButtonPressed, this));
    QObject::connect(rotPosXButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::moveButtonReleased, this));

    QObject::connect(rotNegXButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::negRXButtonPressed, this));
    QObject::connect(rotNegXButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::moveButtonReleased, this));

    QObject::connect(rotPosYButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::posRYButtonPressed, this));
    QObject::connect(rotPosYButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::moveButtonReleased, this));

    QObject::connect(rotNegYButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::negRYButtonPressed, this));
    QObject::connect(rotNegYButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::moveButtonReleased, this));

    QObject::connect(rotPosZButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::posRZButtonPressed, this));
    QObject::connect(rotPosZButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::moveButtonReleased, this));

    QObject::connect(rotNegZButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::negRZButtonPressed, this));
    QObject::connect(rotNegZButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::moveButtonReleased, this));
}

void RobotManualMoveLayout::connectJointButtons()
{
    QObject::connect(q1PosButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::q1PosButtonPressed, this));
    QObject::connect(q2PosButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::q2PosButtonPressed, this));
    QObject::connect(q3PosButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::q3PosButtonPressed, this));
    QObject::connect(q4PosButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::q4PosButtonPressed, this));
    QObject::connect(q5PosButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::q5PosButtonPressed, this));
    QObject::connect(q6PosButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::q6PosButtonPressed, this));
    QObject::connect(q1NegButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::q1NegButtonPressed, this));
    QObject::connect(q2NegButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::q2NegButtonPressed, this));
    QObject::connect(q3NegButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::q3NegButtonPressed, this));
    QObject::connect(q4NegButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::q4NegButtonPressed, this));
    QObject::connect(q5NegButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::q5NegButtonPressed, this));
    QObject::connect(q6NegButton,&QPushButton::pressed,std::bind(&RobotManualMoveLayout::q6NegButtonPressed, this));

    QObject::connect(q1PosButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::jointButtonReleased, this));
    QObject::connect(q2PosButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::jointButtonReleased, this));
    QObject::connect(q3PosButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::jointButtonReleased, this));
    QObject::connect(q4PosButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::jointButtonReleased, this));
    QObject::connect(q5PosButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::jointButtonReleased, this));
    QObject::connect(q6PosButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::jointButtonReleased, this));
    QObject::connect(q1NegButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::jointButtonReleased, this));
    QObject::connect(q2NegButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::jointButtonReleased, this));
    QObject::connect(q3NegButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::jointButtonReleased, this));
    QObject::connect(q4NegButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::jointButtonReleased, this));
    QObject::connect(q5NegButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::jointButtonReleased, this));
    QObject::connect(q6NegButton,&QPushButton::released,std::bind(&RobotManualMoveLayout::jointButtonReleased, this));
}


void RobotManualMoveLayout::q1PosButtonPressed()
{
    jointButtonPressed(0,1);
}

void RobotManualMoveLayout::q2PosButtonPressed()
{
    jointButtonPressed(1,1);
}

void RobotManualMoveLayout::q3PosButtonPressed()
{
    jointButtonPressed(2,1);
}

void RobotManualMoveLayout::q4PosButtonPressed()
{
    jointButtonPressed(3,1);
}

void RobotManualMoveLayout::q5PosButtonPressed()
{
    jointButtonPressed(4,1);
}

void RobotManualMoveLayout::q6PosButtonPressed()
{
    jointButtonPressed(5,1);
}

void RobotManualMoveLayout::q1NegButtonPressed()
{
    jointButtonPressed(0,-1);
}

void RobotManualMoveLayout::q2NegButtonPressed()
{
    jointButtonPressed(1,-1);
}

void RobotManualMoveLayout::q3NegButtonPressed()
{
    jointButtonPressed(2,-1);
}

void RobotManualMoveLayout::q4NegButtonPressed()
{
    jointButtonPressed(3,-1);
}

void RobotManualMoveLayout::q5NegButtonPressed()
{
    jointButtonPressed(4,-1);
}

void RobotManualMoveLayout::q6NegButtonPressed()
{
    jointButtonPressed(5,-1);
}


void RobotManualMoveLayout::coordButtonPressed(int axis, int sign)
{
    Eigen::RowVectorXd operationalVelocity(6);
    operationalVelocity << 0,0,0,0,0,0;
    operationalVelocity(axis) = (sign)*velocityLineEdit->text().toDouble()/1000;
    mRobotInterface->robot->move(corah::MotionType::speedl,operationalVelocity,accelerationLineEdit->text().toDouble(),0,timeLineEdit->text().toDouble(),0);
}

void RobotManualMoveLayout::jointButtonPressed(int joint,int sign)
{
    Eigen::RowVectorXd jointVelocity(6);
    jointVelocity << 0,0,0,0,0,0;
    jointVelocity(joint)=(sign)*velocityLineEdit->text().toDouble()/1000;
    mRobotInterface->robot->move(corah::MotionType::speedj,jointVelocity,accelerationLineEdit->text().toDouble(),0,timeLineEdit->text().toDouble(),0);
}

void RobotManualMoveLayout::rotButtonPressed(int angle, int sign)
{
    Eigen::RowVectorXd operationalVelocity(6);
    operationalVelocity << 0,0,0,0,0,0;
    operationalVelocity(angle+3)=(sign)*velocityLineEdit->text().toDouble()/1000;
    mRobotInterface->robot->move(corah::MotionType::speedl,operationalVelocity,accelerationLineEdit->text().toDouble(),0,timeLineEdit->text().toDouble(),0);
}

void RobotManualMoveLayout::posZButtonPressed()
{
    coordButtonPressed(2,1);
}

void RobotManualMoveLayout::negZButtonPressed()
{
    coordButtonPressed(2,-1);
}

void RobotManualMoveLayout::posYButtonPressed()
{
    coordButtonPressed(1,1);
}

void RobotManualMoveLayout::negYButtonPressed()
{
    coordButtonPressed(1,-1);
}

void RobotManualMoveLayout::posXButtonPressed()
{
    coordButtonPressed(0,1);
}

void RobotManualMoveLayout::negXButtonPressed()
{
    coordButtonPressed(0,-1);
}

void RobotManualMoveLayout::posRXButtonPressed()
{
    rotButtonPressed(0,1);
}

void RobotManualMoveLayout::negRXButtonPressed()
{
    rotButtonPressed(0,-1);
}

void RobotManualMoveLayout::posRYButtonPressed()
{
    rotButtonPressed(1,1);
}

void RobotManualMoveLayout::negRYButtonPressed()
{
    rotButtonPressed(1,-1);
}

void RobotManualMoveLayout::posRZButtonPressed()
{
    rotButtonPressed(2,1);
}

void RobotManualMoveLayout::negRZButtonPressed()
{
    rotButtonPressed(2,-1);
}

void RobotManualMoveLayout::moveButtonReleased()
{
    mRobotInterface->robot->stopMove(corah::MotionType::stopl, accelerationLineEdit->text().toDouble());
}

void RobotManualMoveLayout::jointButtonReleased()
{
    mRobotInterface->robot->stopMove(corah::MotionType::stopj, accelerationLineEdit->text().toDouble());
}

}