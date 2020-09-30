#ifndef _INITIALALIGNMENT_H_
#define _INITIALALIGNMENT_H_

#include "common.h"

class Alignment
{
    public:
    Alignment() { }

    Alignment(const string path);

    Alignment(const VecVector3d Gyro, const VecVector3d Accel);

    public:
    /*******************************************
     * function: to calculate mean observations
     * @return status code
    *******************************************/
    bool CalculateMean();

    /*********************************************
     * function: to do static alignment algorithm
     *           and calculate the Cbn
     * @return status code
    *********************************************/
    bool StaticAlignmentMean();

    /****************************************
     * to calculate the mean euler angle via Cbn
     * @param C     [in]   Cbn
     * @param Euler [out] 
     * @return status code
    ****************************************/
    bool CalculateEuler(const Matrix3d C, double* Euler);

    /**********************************
     * to calculate the euler angle
     * via Cbn epoch by epoch
     * @return status code
    **********************************/
    bool StaticAlignmentEpoch();

    public:
    Vector3d* GetMean();

    Matrix3d GetCbn();

    double* GetEulerMean();

    VecVector3d GetEulerEpoch();

    private:
    /*************************************
     * to store the origin data from file
     * OriginData[0]    Accel
     * OriginData[1]    Gyro
    *************************************/
    VecVector3d OriginData[2];

    private:
    /************************************
     * to store the mean observation
     * Mean[0]    Accel
     * Mean[1]    Gyro
    ************************************/
    Vector3d Mean[2];

    /*************
     * 姿态变换矩阵
    *************/
    Matrix3d Cbn;

    /***********************
     * to store euler angle
     * Euler[0]  yaw
     * Euler[1]  pitch
     * Euler[2]  roll
    ***********************/
    double Euler_mean[3];

    /*********************************
     * calculate euler epoch by epoch
    *********************************/
    VecVector3d Euler_epoch;
};

#endif