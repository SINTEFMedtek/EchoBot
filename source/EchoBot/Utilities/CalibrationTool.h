#ifndef ECHOBOT_CALIBRATIONTOOL_H
#define ECHOBOT_CALIBRATIONTOOL_H

#include <eigen3/Eigen/Dense>
#include "EchoBot/Core/SmartPointers.h"


namespace echobot {

class CalibrationTool {
    ECHOBOT_OBJECT(CalibrationTool)

    public:
        void calibrate();

        Eigen::Affine3d get_rMb(){ return m_rMb;};
        Eigen::Affine3d get_eeMt(){ return m_eeMt;};
        Eigen::Affine3d get_tMus(){ return m_tMus;};

    private:
        Eigen::Affine3d m_rMb, m_eeMt, m_tMus;

        CalibrationTool(){};
        std::weak_ptr<CalibrationTool> mPtr;
};

}


#endif //ECHOBOT_CALIBRATIONTOOL_H
