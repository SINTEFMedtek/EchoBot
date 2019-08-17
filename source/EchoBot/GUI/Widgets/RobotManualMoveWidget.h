#ifndef ROBOTMANUALMOVEWIDGET_H
#define ROBOTMANUALMOVEWIDGET_H

#include <mutex>

#include <QLineEdit>
#include <QPushButton>
#include <QGridLayout>
#include <QScrollBar>
#include <QButtonGroup>
#include <QtWidgets/QTabWidget>

#include "EchoBot/Interfaces/RobotInterface.h"

/**
 * Implementation of Manual motion GUI tab.
 *
 *
 * \author Andreas Østvik
 */


namespace echobot
{

class RobotManualMoveWidget : public QWidget
{
    Q_OBJECT

    public:
        RobotManualMoveWidget(RobotInterface::pointer robotInterface, int widgetWidth=540, int widgetHeight=500);
        virtual ~RobotManualMoveWidget();

    private:
        void setupWidget();
        void setupConnections();

        int mWidgetWidth;
        int mWidgetHeight;

        QBoxLayout *mainLayout;

        RobotInterface::pointer mRobotInterface;

        void connectMovementButtons();
        void connectJointButtons();

        void setMoveToolLayout(QVBoxLayout *vLayout);
        void setMoveSettingsWidget(QVBoxLayout *vLayout);
        void setCoordInfoWidget(QVBoxLayout *vLayout);
        void setJointMoveWidget(QVBoxLayout *vLayout);

        QPushButton *negZButton, *posZButton, *posXButton, *negYButton, *posYButton, *negXButton;
        QPushButton *rotNegZButton, *rotPosZButton, *rotPosXButton, *rotNegYButton, *rotPosYButton, *rotNegXButton;

        QLineEdit *xPosLineEdit, *yPosLineEdit, *zPosLineEdit;
        QLineEdit *rxLineEdit, *ryLineEdit, *rzLineEdit;

        QLineEdit *q1LineEdit, *q2LineEdit, *q3LineEdit;
        QLineEdit *q4LineEdit, *q5LineEdit, *q6LineEdit;

        QPushButton *q1PosButton, *q1NegButton;
        QPushButton *q2PosButton, *q2NegButton;
        QPushButton *q3PosButton, *q3NegButton;
        QPushButton *q4PosButton, *q4NegButton;
        QPushButton *q5PosButton, *q5NegButton;
        QPushButton *q6PosButton, *q6NegButton;

        QButtonGroup *linearMotionButtons, *rotationMotionButtons, *jointConfigurationButtons;

        QLineEdit *accelerationLineEdit, *velocityLineEdit, *timeLineEdit;

        void coordButtonPressed(int axis,int sign);
        void rotButtonPressed(int axis,int sign);
        void jointButtonPressed(int joint,int sign);

        void setAutoRepeat(bool isRepeated, QButtonGroup *buttons);
        void setMaximumWidth(int width, QButtonGroup *buttons);
        void setupLayout();

        std::mutex mUpdateMutex;

    private slots:
        void moveButtonReleased();
        void jointButtonReleased();

        void posZButtonPressed();
        void negZButtonPressed();
        void posYButtonPressed();
        void negYButtonPressed();
        void posXButtonPressed();
        void negXButtonPressed();
        void posRXButtonPressed();
        void negRXButtonPressed();
        void posRYButtonPressed();
        void negRYButtonPressed();
        void posRZButtonPressed();
        void negRZButtonPressed();

        void q1PosButtonPressed();
        void q2PosButtonPressed();
        void q3PosButtonPressed();
        void q4PosButtonPressed();
        void q5PosButtonPressed();
        void q6PosButtonPressed();
        void q1NegButtonPressed();
        void q2NegButtonPressed();
        void q3NegButtonPressed();
        void q4NegButtonPressed();
        void q5NegButtonPressed();
        void q6NegButtonPressed();

        void updatePositions();
};

} // end namespace echobot

#endif // ROBOTMANUALMOVEWIDGET_H
