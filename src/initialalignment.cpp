#include "../include/initialalignment.h"

Alignment::Alignment(const string path)
{
    bool flag = ReadLinsData(path, this->OriginData[0], this->OriginData[1]);
    if(!flag)
        cout << "read file error!!!" << endl;
}

Alignment::Alignment(const VecVector3d Gyro, const VecVector3d Accel)
{
    this->OriginData[0] = Accel;
    this->OriginData[1] = Gyro;
}

bool Alignment::CalculateMean()
{
    for(int i = 0; i < 2; ++i)
    {
        this->Mean[i] = Vector3d::Zero();
        try
        {
            for(auto data : this->OriginData[i])
            {
                this->Mean[i][0] += (data[0]);
                this->Mean[i][1] += (data[1]);
                this->Mean[i][2] += (data[2]);
            }
            this->Mean[i] /= this->OriginData[i].size();
            cout << OriginData[i].size() << endl; 
        }
        catch(const std::exception& e)
        {
            std::cout << e.what() << '\n';
            return false;
        }
    }
    return true;
}

bool Alignment::StaticAlignmentMean()
{
    this->CalculateMean();
    cout << this->Mean[0] << endl;
    Vector3d gn(0, 0, -GRAVITY);
    Vector3d Omega_n(OMEGA * cos(Deg2Rad(PHI)), 0, -OMEGA * sin(Deg2Rad(PHI)));
    Vector3d vn = gn.cross(Omega_n);
    Vector3d vb = this->Mean[0].cross(this->Mean[1]);

    Matrix3d reference;
    reference.block(0, 0, 3, 1) = gn;
    reference.block(0, 1, 3, 1) = Omega_n;
    reference.block(0, 2, 3, 1) = vn;
    
    Matrix3d observation;
    observation.block(0, 0, 3, 1) = this->Mean[0];
    observation.block(0, 1, 3, 1) = this->Mean[1];
    observation.block(0, 2, 3, 1) = vb;
    try
    {
        Matrix3d Cnb = reference.transpose().inverse() * observation.transpose();
        this->Cbn = Cnb;
        for(int i = 0; i < 30; ++i)
        {
            auto temp = (this->Cbn + (Cbn.transpose()).inverse()) * 0.5;
            this->Cbn = temp;
        }

        this->CalculateEuler(Cbn, this->Euler_mean);
    }
    catch(const std::exception& e)
    {
        std::cout << e.what() << '\n';
        return false;
    }

    return true;
}

bool Alignment::CalculateEuler(const Matrix3d C, double* Euler)
{
    Euler[2] = atan2(C(1, 0), C(0, 0));                       // roll
    Euler[1] = atan2(-C(2, 0), sqrt(1 -  C(2, 0) * C(2, 0))); // pitch
    Euler[0] = atan2(C(2, 1), C(2, 2));                       // yaw

    return true;
}

Vector3d* Alignment::GetMean()
{
    return this->Mean;
}

Matrix3d Alignment::GetCbn()
{
    return this->Cbn;
}

double* Alignment::GetEulerMean()
{
    return this->Euler_mean;
}

bool Alignment::StaticAlignmentEpoch(int duration)
{
    if(OriginData[0].size() != OriginData[1].size())
        return false;

    Vector3d gn(0, 0, -GRAVITY);
    Vector3d Omega_n(OMEGA * cos(Deg2Rad(PHI)), 0, -OMEGA * sin(Deg2Rad(PHI)));
    Vector3d vn = gn.cross(Omega_n);
    Matrix3d reference;
    reference.block(0, 0, 3, 1) = gn;
    reference.block(0, 1, 3, 1) = Omega_n;
    reference.block(0, 2, 3, 1) = vn;

    int count = 0;
    Vector3d gb = Vector3d::Zero();
    Vector3d Omega_b = Vector3d::Zero();
    for(int i = 0; i < this->OriginData[0].size(); ++i)
    {
        if(count < duration)
        {
            gb += OriginData[0][i];
            Omega_b += OriginData[1][i];
            count ++;
            continue;
        }
        Vector3d vb = (gb).cross(Omega_b);
        Matrix3d observation;
        observation.block(0, 0, 3, 1) = gb;
        observation.block(0, 1, 3, 1) = Omega_b;
        observation.block(0, 2, 3, 1) = vb;

        try
        {
            Matrix3d Cnb = observation * reference.inverse();
            Matrix3d C = Cnb.transpose();
            for(int i = 0; i < 30; ++i)
            {
                auto temp = (C + (C.transpose()).inverse()) * 0.5;
                C = temp;
            }
            double Euler[3];
            bool flag = this->CalculateEuler(C, Euler);
            if(!flag)
            {
                cout << "calculate Euler wrong" << endl;
                return false;
            }
            this->Euler_epoch.push_back(Vector3d(Euler[0], Euler[1], Euler[2]));
        }
        catch(const std::exception& e)
        {
            std::cout << e.what() << '\n';
            return false;
        }
        gb = Vector3d::Zero();
        Omega_b = Vector3d::Zero();
        count = 0;
    }

    return true;
}

VecVector3d Alignment::GetEulerEpoch()
{
    return this->Euler_epoch;
}
