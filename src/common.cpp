#include "../include/common.h"
#include <algorithm>
#include <fstream>
#include <string>
#include <iomanip>

// WGS84基椭球参数
const double a_w = 6378137;
const double b_w = 6356752.3142;

// CGCS2000 椭球参数
const double a_c = 6378137;
const double b_c = 6356752.3141;

// GRS
const double a_g = 6378137.0;
const double b_g = 6356752.3141;
const double gama_b_grs = 9.8321863685;
const double gama_a_grs = 9.7803267715;
const double GM_grs = 3.986005e14;

bool ReadASC(string ASCPath, VecVector3d &Accel, VecVector3d &Gyro)
{
    ifstream ASCFile;
    ASCFile.open(ASCPath);
    if(!ASCFile)
    {
        cerr << "open error!" << endl;
        return false;
    }

    while(!ASCFile.eof())
    {
        /* 去标点处理 */
        string line;
        stringstream buff;
        getline(ASCFile, line);
        replace(line.begin(), line.end(), ',', ' ');
        replace(line.begin(), line.end(), ';', ' ');
        replace(line.begin(), line.end(), '*', ' ');
        buff << line;

        /* 数据存入内存 */
        Vector3d gyro, accel;
        /* 本次实验暂时未使用 */
        int week, status;
        double SOW;
        string head;
        buff >> head >> week >> SOW >> week >> SOW >> status >> accel[2] >> accel[1] >> accel[0]
             >> gyro[2] >> gyro[1] >> gyro[0];
        /* Y轴的输出是加了一个负号 */
        accel[1] = -accel[1];
        
        if(status != 77)
            cout << "observations are not operable!\n" << "time: " << week << ", " << SOW << endl;
            
        accel *= ACCEL_SCALE;
        gyro *= GYRO_SCALE;
        Accel.push_back(accel);
        Gyro.push_back(gyro);
    }
    
    return true;
}

double Deg2Rad(const double deg)
{
    double sec = deg * 3600.0;
    double rad = sec / SECRAD;

    return rad;
}

double Rad2Deg(const double rad)
{
    double sec = rad * SECRAD;
    double deg = sec / 3600.0;

    return deg;
}

bool Compensate(const Matrix34d M, const VecVector3d OriginOuput, VecVector3d &Compensated)
{
    Vector3d bias = M.block(0, 3, 3, 1);
    Matrix3d M1 = M.block(0, 0, 3, 3);

    for(auto data : OriginOuput)
    {
        Vector3d temp = data - bias;
        temp = M1.inverse() * temp;
        Compensated.push_back(temp);
    }
    return true;
}

bool ReadLinsData(const string DataFile, VecVector3d &Accel, VecVector3d &Gyro)
{
    ifstream in;
    in.open(DataFile);
    if(!in)
    {
        cout << "open file error" << endl;
        return false;
    }

    while(!in.eof())
    {
        string line;
        stringstream buff;
        getline(in, line);
        if(line[0] == 'm')
            continue;
        long int week, SOW;
        Vector3d Accel_temp, Gyro_temp;

        buff << line;
        buff >> week >> SOW >> Gyro_temp[1] >> Gyro_temp[0] >> Gyro_temp[2]
             >> Accel_temp[1] >> Accel_temp[0] >> Accel_temp[2];

        Accel_temp[0] = (Accel_temp[0] * GRAVITY);
        Accel_temp[1] = (Accel_temp[1] * GRAVITY);
        Accel_temp[2] = -(Accel_temp[2] * GRAVITY);
        
        Gyro_temp[0] = Deg2Rad((Gyro_temp[0])) / 3600.0;
        Gyro_temp[1] = Deg2Rad((Gyro_temp[1])) / 3600.0;
        Gyro_temp[2] = Deg2Rad(-(Gyro_temp[2])) / 3600.0;

        Accel.push_back(Accel_temp);
        Gyro.push_back(Gyro_temp);
    }

    return true;
}

bool ReadBinaryData(FILE* fp, Output &p)
{
    if(!fread(&p, sizeof(p), 1, fp))
        return false;
    p.az = -p.az;
    return true;
}

bool DetectStaticData(FILE* fp, VecVector3d &gyro, VecVector3d &accel)
{
    while (!feof(fp))
    {
        Output p;
        if(!fread(&p, sizeof(p), 1, fp))
            return false;
        if(abs(p.gx) > 8e-4)
            return true;
        // cout << p.time << endl;
        gyro.push_back(Vector3d(Deg2Rad(p.gy), Deg2Rad(p.gx), Deg2Rad(-p.gz)));
        accel.push_back(Vector3d(p.ay, p.ax, -p.az));
    }
}

ostream & operator<<(ostream &out, const Euler euler)
{
    out << setprecision(5) << Rad2Deg(euler.yaw) << "   "
        << Rad2Deg(euler.pitch) << "   " << Rad2Deg(euler.roll);
}

Quaterniond Eu2Qu(Euler eu)
{
    // eu.pitch = Deg2Rad(eu.pitch);
    // eu.roll  = Deg2Rad(eu.roll);
    // eu.yaw   = Deg2Rad(eu.yaw);
    double w = cos(eu.roll / 2.0) * cos(eu.pitch / 2.0) * cos(eu.yaw / 2.0) + 
               sin(eu.roll / 2.0) * sin(eu.pitch / 2.0) * sin(eu.yaw / 2.0);
    double x = sin(eu.roll / 2.0) * cos(eu.pitch / 2.0) * cos(eu.yaw / 2.0) - 
               cos(eu.roll / 2.0) * sin(eu.pitch / 2.0) * sin(eu.yaw / 2.0);
    double y = cos(eu.roll / 2.0) * sin(eu.pitch / 2.0) * cos(eu.yaw / 2.0) + 
               sin(eu.roll / 2.0) * cos(eu.pitch / 2.0) * sin(eu.yaw / 2.0);
    double z = cos(eu.roll / 2.0) * cos(eu.pitch / 2.0) * sin(eu.yaw / 2.0) - 
               sin(eu.roll / 2.0) * sin(eu.pitch / 2.0) * cos(eu.yaw / 2.0);
    return Quaterniond(w, x, y, z);
}

BLH::BLH(double B_, double L_, double H_)
{
    this->B = B_;
    this->L = L_;
    this->H = H_;
}

ostream & operator<<(ostream &out, const BLH blh)
{
    out << "(" << blh.B << ", " << blh.L << ", " << blh.H << ")" << endl;
    return out;
}

ELLIPSOID::ELLIPSOID()
{
    this->a = a_c;
    this->b = b_c;
    this->c = this->a * this->a / this->b;
    this->alpha = (this->a - this->b) / this->a;
    this->e2 = (this->a * this->a - this->b * this->b) / (this->a * this->a);
    this->e1_2 = (this->a * this->a - this->b * this->b) / (this->b * this->b);
    this->gama_a = gama_a_grs;
    this->gama_b = gama_b_grs;
}

ELLIPSOID::ELLIPSOID(double a_, double b_)
{
    this->a = a_;
    this->b = b_;
    this->c = this->a * this->a / this->b;
    this->alpha = (this->a - this->b) / this->a;
    this->e2 = (this->a * this->a - this->b * this->b) / (this->a * this->a);
    this->e1_2 = (this->a * this->a - this->b * this->b) / (this->b * this->b);
    this->gama_a = gama_a_grs;
    this->gama_b = gama_b_grs;
    this->GM = GM_grs;
}

double RM(double phi, ELLIPSOID type)
{
    double sinPhi = sin((phi));
    double up = type.a * (1 - type.e2);
    double down = pow(1 - type.e2 * sinPhi * sinPhi, 1.5);
    return up / down;
}

double RN(double phi, ELLIPSOID type) {
    double sinPhi = sin((phi));
    double down = sqrt(1 - type.e2 * sinPhi * sinPhi);
    return type.a / down;
}

int GetOutput(FILE* fp, Output &op) {
    if(!fread(&op, sizeof(op), 1, fp))
        return 1;
    // op.az = -op.az;
    return 0;
}

double GetGravity(ELLIPSOID type, BLH blh) {
    double m = OMEGA * OMEGA * type.a * type.a * type.b / type.GM;

    double up_phi = type.a * type.gama_a * cos(blh.B) * cos(blh.B) +
                    type.b * type.gama_b * sin(blh.B) * sin(blh.B);
    double down_phi = sqrt(type.a * type.a * cos(blh.B) * cos(blh.B) + 
                           type.b * type.b * sin(blh.B) * sin(blh.B));
    double phi = up_phi / down_phi;

    double right = 1.0 - 2.0 / type.a * 
                   (1.0 + type.alpha + m - 2.0 * type.alpha * sin(blh.B) * sin(blh.B)) *
                   blh.H + 3.0 / (type.a * type.a) * blh.H * blh.H;
    
    return (phi * right);     
}

Matrix3d Eu2Ro(Euler eu) {
    Matrix3d rotation;
    rotation(0, 0) = cos(eu.pitch) * cos(eu.yaw);
    rotation(0, 1) = -cos(eu.roll) * sin(eu.yaw) + sin(eu.roll) * sin(eu.pitch) * cos(eu.yaw);
    rotation(0, 2) = sin(eu.roll) * sin(eu.yaw) + cos(eu.roll) * sin(eu.pitch) * cos(eu.yaw);

    rotation(1, 0) = cos(eu.pitch) * sin(eu.yaw);
    rotation(1, 1) = cos(eu.roll) * cos(eu.yaw) + sin(eu.roll) * sin(eu.pitch) * sin(eu.yaw);
    rotation(1, 2) = -sin(eu.roll) * cos(eu.yaw) + cos(eu.roll) * sin(eu.pitch) * sin(eu.yaw);

    rotation(2, 0) = -sin(eu.pitch);
    rotation(2, 1) = sin(eu.roll) * cos(eu.pitch);
    rotation(2, 2) = cos(eu.roll) * cos(eu.pitch);

    return rotation;
}

Matrix3d Vector2Matrix(Vector3d vec) {
    Matrix3d m = Matrix3d::Zero();
    m(0, 1) = -vec(2, 0);
    m(0, 2) = vec(1, 0);

    m(1, 0) = vec(2, 0);
    m(1, 2) = -vec(0, 0);

    m(2, 0) = -vec(1, 0);
    m(2, 1) = vec(0, 0);

    return m;
}

bool CalculateEuler(const Matrix3d C, Vector3d &Euler)
{
    Euler[0] = atan2(C(1, 0), C(0, 0));                       // yaw
    Euler[1] = atan2(-C(2, 0), sqrt(1 -  C(2, 0) * C(2, 0))); // pitch
    Euler[2] = atan2(C(2, 1), C(2, 2));                       // roll

    return true;
}

Quaterniond 
Vec2Qua(Vector3d vec) {
    double angle = 0.5 * vec.norm();
    double coe = sin(angle) / angle;

    double w = cos(angle);
    double a = coe * vec(0, 0) * 0.5;
    double b = coe * vec(1, 0) * 0.5;
    double c = coe * vec(2, 0) * 0.5;

    return Quaterniond(w, a, b, c);
}

