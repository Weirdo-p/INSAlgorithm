/****************************************
 * to declare some functions may be used
 * @author XUZHUO WHU
 * 2020 09 24
****************************************/

#ifndef _COMMON_H_
#define _COMMON_H_

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>

#define PHI 30.5278 // in deg
#define GRAVITY 9.7936174
#define OMEGA 7.292115e-5 // self rotation
#define _T_ 0.01 // frequency

#define GYRO_SCALE 0.1 / (3600.0 * 256.0)
#define ACCEL_SCALE 0.05 / 32768.0
#define PI 3.14159265358979323846
#define SECRAD  (180.0 * 3600 / PI)  // 206265

using namespace std;
using namespace Eigen;

typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef Matrix<double, 3, 4> Matrix34d;
typedef Matrix<double, 4, 6> Matrix46d;

/********************************************
 * function: to read data from ASC files
 * @param ASCPath [in] the path of ASC file
 * @param Accel [out] the Accel output
 * @param Gyro [out] the Gyro output
 * @return flag  status code
 *               true successful
 *               false deadly error
********************************************/
bool ReadASC(string ASCPath, VecVector3d &Accel, VecVector3d &Gyro);

/************************************
 * function: to transform deg to rad
 * @param deg  double  degree
 * @return  rad  double  rad
************************************/
double Deg2Rad(const double deg);

/************************************
 * function: to transform rad to deg
 * @param  rad  double rad
 * @return  deg  double degree
************************************/
double Rad2Deg(const double rad);

/*********************************************************
 * function: to compensate accel output
 * @param M [in] 零偏、比例因子、交轴耦合矩阵
 * @param AccelOriginOutput [in] origin output of accel
 * @param AccelCompensated [out] compensated accel output
 * @return status code
*********************************************************/
bool Compensate(const Matrix34d M, const VecVector3d AccelOriginOuput, VecVector3d &AccelCompensated);
#endif