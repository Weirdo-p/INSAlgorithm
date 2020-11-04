#include "calibrateAccel.h"
#include "common.h"

#include <fstream>
#include <Eigen/Dense>

CalibrateAccel::CalibrateAccel(string* OutputFile)
{
    VecVector3d gyro_temp;
    for(int i = 0; i < 6; ++i)
    {
        try
        {
            ReadASC(OutputFile[i], this->Accel[i], gyro_temp);
        }
        catch(...)
        {
            cout << "数组越界，请检查！" << endl;
        }
        vector<Vector3d, Eigen::aligned_allocator<Vector3d>>().swap(gyro_temp);
    }

    this->CalculateMean(this->MeanOutput);
}

bool CalibrateAccel::CalculateMean(Vector3d* Mean)
{
    for(int i = 0; i < 6; ++i)
    {
        try
        {
            Mean[i] = Vector3d::Zero();
            for(auto data : this->Accel[i])
            {
                Mean[i][0] += data[0];
                Mean[i][1] += data[1];
                Mean[i][2] += data[2];
            }
            Mean[i] /= Accel[i].size();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }
    }

    return true;
}

VecVector3d* CalibrateAccel::GetAccelOrigin()
{
    return this->Accel;
}

bool CalibrateAccel::CalculateM()
{
    Matrix46d A;
    A.block(0, 0, 4, 1) = Vector4d(GRAVITY * _T_, 0, 0, 1);
    A.block(0, 1, 4, 1) = Vector4d(-GRAVITY * _T_, 0, 0, 1);
    A.block(0, 2, 4, 1) = Vector4d(0, GRAVITY * _T_, 0, 1);
    A.block(0, 3, 4, 1) = Vector4d(0, -GRAVITY * _T_, 0, 1);
    A.block(0, 4, 4, 1) = Vector4d(0, 0, GRAVITY * _T_, 1);
    A.block(0, 5, 4, 1) = Vector4d(0, 0, -GRAVITY * _T_, 1);

    Matrix<double, 3, 6> L;
    for(int i = 0; i < 6; ++i)
        L.block(0, i, 3, 1) = this->MeanOutput[i];

    this->M = L * A.transpose() * (A * A.transpose()).inverse();

    return true;
}

Matrix34d CalibrateAccel::GetM()
{
    return this->M;
}

