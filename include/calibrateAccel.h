/********************************************
 * this file used for algorithm to calibrate
 * accel with 6-position method
 * @author XUZHUO WHU
********************************************/

#ifndef _CALIBRATEACCEL_H_
#define _CALIBRATEACCEL_H_

#include "common.h"

class CalibrateAccel
{
    public:
    CalibrateAccel() { }

    /*****************************
     * OutputFile[0]  x_up
     * OutputFile[1]  x_down
     * OutputFile[2]  y_up
     * OutputFile[3]  y_down
     * OutputFile[4]  z_up
     * OutputFile[5]  z_down
    *****************************/
    CalibrateAccel(string* OutputFile);

    public:
    /***************************************************
     * function: to calculate mean output of each axis
     * Mean[0] [out] x_up_mean
     * Mean[1] [out] x_down_mean
     * Mean[2] [out] y_up_mean
     * Mean[3] [out] y_down_mean
     * Mean[4] [out] z_up_mean
     * Mean[5] [out] z_down_mean
     * @return flag status code
    ***************************************************/
    bool CalculateMean(Vector3d* Mean);


    /*************************************************
     * function: to calculate matrix M by using 
     *           least square method
     * @return status code
    *************************************************/
    bool CalculateM();

    public:
    /*********************
     * function: return M
     * @param return M
    *********************/
    Matrix34d GetM();

    /********************************************
     * function: return origin obs of Accel
     * Accel[0] [out] x_up_mean
     * Accel[1] [out] x_down_mean
     * Accel[2] [out] y_up_mean
     * Accel[3] [out] y_down_mean
     * Accel[4] [out] z_up_mean
     * Accel[5] [out] z_down_mean
     * @return Accel
    ********************************************/
    VecVector3d* GetAccelOrigin();

    public:
    friend bool ReadASC(string ASCPath, VecVector3d &Accel, VecVector3d &Gyro);

    // members we have known
    private:
    VecVector3d Accel[6];

    // members need to calculate
    private:
    Matrix34d M;
    Vector3d MeanOutput[6];
};


#endif