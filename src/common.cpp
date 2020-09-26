#include "../include/common.h"
#include <algorithm>
#include <fstream>
#include <string>


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
        gyro[0] = abs(gyro[0]);
        gyro[1] = abs(gyro[1]);
        gyro[2] = abs(gyro[2]);
        
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
    ofstream out;
    out.open("/home/weirdo/Documents/coding/INSAlgorithm/origin_xPosi.txt");
    ofstream out1;
    out1.open("/home/weirdo/Documents/coding/INSAlgorithm/compensated_xPosi.txt");
    Vector3d bias = M.block(0, 3, 3, 1);
    Matrix3d M1 = M.block(0, 0, 3, 3);
    cout << bias << endl;

    for(auto data : OriginOuput)
    {
        out << data[0] << "," << data[1] << "," << data[2] << endl;
        Vector3d temp = data - bias;
        temp = M1.inverse() * temp;
        out1 << temp[0] << "," << temp[1] << "," << temp[2] << endl;
        Compensated.push_back(temp);
    }
    return true;
}