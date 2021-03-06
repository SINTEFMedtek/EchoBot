#include "CalibrationTool.h"

#include <Eigen/Dense>
#include <fstream>
#include <iostream>

namespace echobot {


void CalibrationTool::calibrate() {

}

Eigen::Affine3d CalibrationTool::loadCalFile(std::string path) {
    auto calMat = Eigen::Affine3d::Identity();
    std::ifstream inFile;
    inFile.open(path);

    if(inFile.is_open())
    {
        for(int row = 0; row < calMat.rows(); row++){
            for(int col = 0; col < calMat.cols(); col++)
            {
                double item = 0;
                inFile >> item;
                calMat(row, col) = item;
            }
        }
        inFile.close();
    }
    return calMat;
}

void CalibrationTool::saveCalFile(std::string path, Eigen::Affine3d calMat) {
    std::ofstream outFile;
    outFile.open(path);

    if(outFile.is_open()){
        outFile << calMat.matrix();
        outFile.close();
    }
}

CalibrationTool::CalibrationTool() {
    mCalibrationFilePath = Config::getConfigPath() + "calibration/";
    m_rMb = loadCalFile(mCalibrationFilePath + "camMbase.cal");
    m_eeMt = loadCalFile(mCalibrationFilePath + "eeMtool.cal");
    m_tMus = loadCalFile(mCalibrationFilePath + "toolMus.cal");
    m_registration_pcMdata = loadCalFile(mCalibrationFilePath + "registration_pcMdata.cal");
    m_registration_pcMt = loadCalFile(mCalibrationFilePath + "registration_pcMt.cal");
    std::cout << m_registration_pcMdata.matrix() << std::endl;
}

void CalibrationTool::set_rMb(Eigen::Affine3d mat) {
    m_rMb = mat;
}

void CalibrationTool::set_eeMt(Eigen::Affine3d mat) {
    m_eeMt = mat;
}

void CalibrationTool::set_tMus(Eigen::Affine3d mat) {
    m_tMus = mat;
}

void CalibrationTool::set_registration_pcMdata(Eigen::Affine3d mat) {
    m_registration_pcMdata = mat;
}

void CalibrationTool::set_registration_pcMt(Eigen::Affine3d mat) {
    m_registration_pcMt = mat;
}

Eigen::Affine3d CalibrationTool::get_registration_pcMdata(){
    std::cout << m_registration_pcMdata.matrix() << std::endl;
    return m_registration_pcMdata;
}

}

