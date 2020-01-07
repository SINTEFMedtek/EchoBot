#include "CalibrationTool.h"

#include <Eigen/Dense>


namespace echobot {


void CalibrationTool::calibrate() {
    Eigen::Affine3d rMb = Eigen::Affine3d::Identity();

    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(1.0*M_PI, Eigen::Vector3d::UnitX())*
        Eigen::AngleAxisd(0.0*M_PI, Eigen::Vector3d::UnitY());

    // 0.51*M_PI, 0.96*M_PI, 0.0*M_PI

    rMb.translate(Eigen::Vector3d(600, -550, 1520)); // -500, 370, 1000 (y,x,z)
    rMb.linear() = rMb.linear()*m;

    m_rMb = rMb;

    Eigen::Affine3d eeMt = Eigen::Affine3d::Identity();

    Eigen::Matrix3d rotProbe;
    rotProbe = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())*
               Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX())*
               Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());

    eeMt.translate(Eigen::Vector3d(0,-65,130));
    eeMt.linear() = eeMt.linear()*rotProbe;

    m_eeMt = eeMt;
}


}

