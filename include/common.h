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

// #define PHI 30.4042231878 // in deg
#define PHI 30.5278 // in deg

#define GRAVITY 9.7936174
#define OMEGA 7.292115e-5 // self rotation
#define _T_ 0.01 // frequency

#define GYRO_SCALE            0.1 / (3600.0 * 256.0)
#define ACCEL_SCALE           0.05 / 32768.0
#define PI                    3.14159265358979323846
#define SECRAD                (180.0 * 3600 / PI)  // 206265
#define ZCF                   200

using namespace std;
using namespace Eigen;

typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef Matrix<double, 3, 4> Matrix34d;
typedef Matrix<double, 4, 6> Matrix46d;

extern const double a_g;
extern const double b_g;
extern const double gama_a_grs;
extern const double gama_b_grs;
extern const double GM_grs;


struct Output
{
    double time = 0;                // time for observation
    double gx = 0, gy = 0, gz = 0;  // gyro output
    double ax = 0, ay = 0, az = 0;  // accel output
};

struct Euler
{
    double roll;
    double pitch;
    double yaw;

    Euler() { }

    Euler(double roll, double pitch, double yaw) { this->yaw = yaw; this->roll = roll; this->pitch = pitch; }

    friend ostream & operator<<(ostream &out, const Euler euler);
};

struct Velocity
{
    double vx = 0, vy = 0, vz = 0;

    Velocity() { }

    Velocity(double x, double y, double z) { this->vx = x; this->vy = y; this->vz = z; }
};


/********************************
 * BLH coordinate structure
 * 默认构造0
 * @param B    double  latitude
 * @param L    double  longitude
 * @param H    double  height
********************************/
struct BLH
{
    double B;
    double L;
    double H;

    BLH() { }

    BLH(double B_, double L_, double H_);

    friend ostream & operator<<(ostream &out, const BLH blh);
};

/*********************************
 * Ellipsoid parameters
 * 默认使用WGS84参数
 * @param a  长半轴
 * @param b  短半轴
 * @param c  简化书写而使用的符号
 * @param alpha  扁率
 * @param e2     第一偏心率的二次方
 * @param e1_2   第二偏心率的二次方
*********************************/
struct ELLIPSOID
{
    double a;
    double b;
    double c;
    double alpha;
    double e2;
    double e1_2;
    double gama_a;
    double gama_b;
    double GM;

    ELLIPSOID();
    
    ELLIPSOID(double a_, double b_);
};

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

/********************************************************
 * function: to read the data from the decoded file
 *           using to do static-state initial alignment
 * @param DataFile [in] path of your file
 * @param Accel    [out] Accel output
 * @param Gyro     [out] Gyro output
 * @return status code
********************************************************/
bool ReadLinsData(const string DataFile, VecVector3d &Accel, VecVector3d &Gyro);

/****************************************
 * function: read a line of binary file
 * @param fp [in]  file point
 * @param p  [out] pose in time t
 * @return         status code
****************************************/
bool ReadBinaryData(FILE* fp, Output &p);

/*****************************************
 * function: detect static data to ailgn
 * @param fp    [in]   file point
 * @param gyro  [out]  gyro data
 * @param accel [out]  accel data
 * @return status code
*****************************************/
bool DetectStaticData(FILE* fp, VecVector3d &gyro, VecVector3d &accel);

Quaterniond Eu2Qu(Euler eu);

Matrix3d Eu2Ro(Euler eu);

/**
 * function: to transfer a vector to its symmetrix matrix
*/
Matrix3d Vector2Matrix(Vector3d vec);

double RM(double phi, ELLIPSOID type);

double RN(double phi, ELLIPSOID type);

int GetOutput(FILE* fp, Output &op);

double GetGravity(ELLIPSOID type, BLH blh);

bool CalculateEuler(const Matrix3d C, Vector3d &Euler);

Quaterniond Vec2Qua(Vector3d vec);

Quaterniond Eu2Qua(Euler eu);
#endif