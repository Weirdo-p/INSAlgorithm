/***************************************
 * this file is used to calibrate gyro
 * also with 6-position method
 * @author XUZHUO WHU
 * 2020 09 22
***************************************/

#ifndef _CALIBRATEGYRO_H_
#define _CALIBRATEGYRO_H_

#include "common.h"

class CalibrateGyro
{
    public:
    CalibrateGyro() { }

    /**********************************
     * OutputFile_Kinematic[0]  x_up
     * OutputFile_Kinematic[1]  x_down
     * OutputFile_Kinematic[2]  y_up
     * OutputFile_Kinematic[3]  y_down
     * OutputFile_Kinematic[4]  z_up
     * OutputFile_Kinematic[5]  z_down
     * OutputFile_Static is same as Kinematic
    **********************************/
    CalibrateGyro(string* OutputFile_Kinematic, string* OutputFile_Static);

    public:
    /************************************
     * function: calculate the zero bias
     * using static data
     * @return status code
    ************************************/
    bool CalculateBias();

    /******************************************************
     * function: calculate scale factor and cross couping
     * @return status code
    *******************************************************/
    bool CalculateM();

    /**********************************************************
     * function: to record rotation data and the duration time
     * it will use KINEMATIC data 
     * @param GyroData     [in] 
     * @param axis         [in]  which axis you want to get 
     *                           0  -   x
     *                           1  -   y
     *                           2  -   z
     * @param num           [out] how many data during rotation
     * @param RotationAngle [out]  total rotation angle with specific axis
     * @return status code
    **********************************************************/
    bool CalculateRotation(const VecVector3d GyroData, const int axis, Vector3d &RotationAngle, int &num);

    Matrix34d GetM();

    Vector3d GetBias();

    VecVector3d* GetOrigin_Kinematic();

    VecVector3d* GetOrigin_Static();

    public:
    friend bool ReadASC(string ASCPath, VecVector3d &Accel, VecVector3d &Gyro);

    friend bool Compensate(const Matrix34d M, const VecVector3d OriginOuput, VecVector3d &Compensated);
    
    private:
    /**********************************
     * to store origin KINEMATIC output of Gyro
     * Gyro_Kinematic[0]  x_up
     * Gyro_Kinematic[1]  x_down
     * Gyro_Kinematic[2]  y_up
     * Gyro_Kinematic[3]  y_down
     * Gyro_Kinematic[4]  z_up
     * Gyro_Kinematic[5]  z_down
    **********************************/
    VecVector3d Gyro_Kinematic[6];

    /*******************************************
     * to store origin STATIC output of Gyro
     * Gyro_Static[0]  x_up
     * Gyro_Static[1]  x_down
     * Gyro_Static[2]  y_up
     * Gyro_Static[3]  y_down
     * Gyro_Static[4]  z_up
     * Gyro_Static[5]  z_down
    *******************************************/
    VecVector3d Gyro_Static[6];

    // members need to be calculated
    private:
    Vector3d Bias;

    Matrix34d M;
};


#endif