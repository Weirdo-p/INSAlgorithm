#include "../include/calibrateGyro.h"
#include <fstream>

CalibrateGyro::CalibrateGyro(string* OutputFile_Kinematic, string* OutputFile_Static)
{
    VecVector3d accel_temp;
    for(int i = 0; i < 6; ++i)
    {
        try
        {
            ReadASC(OutputFile_Kinematic[i], accel_temp, this->Gyro_Kinematic[i]);
            ReadASC(OutputFile_Static[i], accel_temp, this->Gyro_Static[i]);
        }
        catch(...)
        {
            cout << "数组越界，请检查！" << endl;
        }
        vector<Vector3d, Eigen::aligned_allocator<Vector3d>>().swap(accel_temp);
    }
    // ofstream out;
    // out.open("/home/weirdo/Downloads/test.txt");
    // for(auto data : Gyro_Kinematic[1])
    // {
    //     out << data[0] << "," << data[1] << "," << data[2] << endl;
    // }
    double test = 0;
    for(auto data : Gyro_Kinematic[0])
    {
        test += data[0];
    }
}

bool CalibrateGyro::CalculateBias()
{
    double mean[6];
    try
    {
        for(int i = 0; i < 6; ++i)
        {
            double sum = 0;
            for(auto data : this->Gyro_Static[i])
            
                if(i <= 1)
                    sum += data[0];
                else if(i >= 2 && i < 4)
                    sum += data[1];
                else if(i >= 4 && i < 6)
                    sum += data[2];
            sum /= Gyro_Static[i].size();
            mean[i] = sum;
        }

        Bias[0] = (mean[0] + mean[1]) / 2.0;
        Bias[1] = (mean[2] + mean[3]) / 2.0;
        Bias[2] = (mean[4] + mean[5]) / 2.0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }

    return true;
}

bool CalibrateGyro::CalculateM()
{
    bool flag = CalculateBias();
    if(!flag)
        return false;
    Vector3d RotationAngle[6];
    int num[6];
    try
    {
        for(int i = 0; i < 6; ++i)
        {
            for(auto data : this->Gyro_Static[i])
                if(i <= 1)
                    this->CalculateRotation(this->Gyro_Kinematic[i], 0, RotationAngle[i], num[i]);
                else if(i >= 2 && i < 4)
                    this->CalculateRotation(this->Gyro_Kinematic[i], 1, RotationAngle[i], num[i]);
                else if(i >= 4 && i < 6)
                    this->CalculateRotation(this->Gyro_Kinematic[i], 2, RotationAngle[i], num[i]);
            cout << RotationAngle[i] << "     " << num[i] << endl << endl;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
    
    double PI_2 = PI * 2;
    M(0, 0) = (RotationAngle[0][0] - RotationAngle[1][0]) / PI_2;   // X axis scale factor
    M(1, 1) = (RotationAngle[2][1] - RotationAngle[3][1]) / PI_2;   // Y axis scale factor
    M(2, 2) = (RotationAngle[4][2] - RotationAngle[5][2]) / PI_2;   // Z axis scale factor

    M(0, 1) = (RotationAngle[2][0] - RotationAngle[3][0]) / PI_2;   // yx
    M(0, 2) = (RotationAngle[4][0] - RotationAngle[5][0]) / PI_2;   // zx

    M(1, 0) = (RotationAngle[0][1] - RotationAngle[1][1]) / PI_2;   // xy
    M(1, 2) = (RotationAngle[4][1] - RotationAngle[5][1]) / PI_2;   // zy

    M(2, 0) = (RotationAngle[0][2] - RotationAngle[1][2]) / PI_2;   // xz
    M(2, 1) = (RotationAngle[2][2] - RotationAngle[3][2]) / PI_2;   // yz

    M.block(0, 0, 3, 3) += Matrix3d::Identity();
    M.block(0, 3, 3, 1) += this->Bias;
    return true;
}

bool CalibrateGyro::CalculateRotation(const VecVector3d GyroData, const int axis, Vector3d &RotationAngle, int &num)
{
    num = 0;
    RotationAngle = Vector3d::Zero();
    try
    {
        for(auto data : GyroData)
        {
            if(abs(data[axis]) < 1e-5)
                continue;
            num ++;
            RotationAngle += data;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }

    if(num == 0)
        return false;

    return true;
}

Matrix34d CalibrateGyro::GetM()
{
    return this->M;
}

Vector3d CalibrateGyro::GetBias()
{
    return this->Bias;
}

VecVector3d* CalibrateGyro::GetOrigin()
{
    return this->Gyro_Kinematic;
}
