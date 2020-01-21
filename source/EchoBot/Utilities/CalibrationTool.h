#ifndef ECHOBOT_CALIBRATIONTOOL_H
#define ECHOBOT_CALIBRATIONTOOL_H

#include <eigen3/Eigen/Dense>
#include "EchoBot/Core/SmartPointers.h"
#include "EchoBot/Core/Config.h"


namespace echobot {

class CalibrationTool {
    ECHOBOT_OBJECT(CalibrationTool)

    public:
        void calibrate();
        Eigen::Affine3d get_rMb(){ return m_rMb;};
        Eigen::Affine3d get_eeMt(){ return m_eeMt;};
        Eigen::Affine3d get_tMus(){ return m_tMus;};

        void set_rMb(Eigen::Affine3d mat);
        void set_eeMt(Eigen::Affine3d mat);
        void set_tMus(Eigen::Affine3d mat);

        std::string getCalibrationFilePath(){return mCalibrationFilePath;};
        void setCalibrationFilePath(std::string path){mCalibrationFilePath = path;};

        static Eigen::Affine3d loadCalFile(std::string path);
        static void saveCalFile(std::string path, Eigen::Affine3d calMat);

    private:
        Eigen::Affine3d m_rMb, m_eeMt, m_tMus;
        std::string mCalibrationFilePath;

        CalibrationTool();
        std::weak_ptr<CalibrationTool> mPtr;
};

}


#endif //ECHOBOT_CALIBRATIONTOOL_H
