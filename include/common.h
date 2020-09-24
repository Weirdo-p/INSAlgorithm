/****************************************
 * to declare some functions may be used
 * @author XUZHUO WHU
 * 2020 09 24
****************************************/

#ifndef _COMMON_H_
#define _COMMON_H_


#include <iostream>
#include <Eigen/Core>
#include <vector>

using namespace std;
using namespace Eigen;

#define GYRO_SCALE 0.1 / (3600.0 * 256.0)
#define ACCEL_SCALE 0.05 / 32768.0

typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;

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

#endif