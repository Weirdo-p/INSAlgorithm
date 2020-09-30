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
    cout << Mean[0] << endl;
    cout << Mean[1] << endl;
    Vector3d gn(0, 0, -GRAVITY);
    Vector3d Omega_n(OMEGA * cos(Deg2Rad(PHI)), 0, -OMEGA * sin(Deg2Rad(PHI)));
    Vector3d vn = gn.cross(Omega_n);
    Vector3d vb = this->Mean[0].cross(this->Mean[1]);

    Matrix3d reference;
    reference.block(0, 0, 3, 1) = gn;
    reference.block(0, 1, 3, 1) = Omega_n;
    reference.block(0, 2, 3, 1) = vn;
    // cout << "ref" << endl << reference.inverse() << endl;
    // reference << -tan(PHI) / GRAVITY, 1.0 / (OMEGA * cos(PHI)), 0,
    //              0, 0, -1.0 / (GRAVITY * OMEGA * cos(PHI)),
    //              -1.0 / GRAVITY, 0, 0;

    Matrix3d observation;
    observation.block(0, 0, 3, 1) = this->Mean[0];
    observation.block(0, 1, 3, 1) = this->Mean[1];
    observation.block(0, 2, 3, 1) = vb;
    cout << "obs" << endl << observation << endl;
    try
    {
        // Matrix3d R;
        // R << -tan(Deg2Rad(PHI)) / GRAVITY, 1.0 / (OMEGA * cos(Deg2Rad(PHI))), 0,
        //      0, 0, -1.0 / (GRAVITY * OMEGA * cos(Deg2Rad(PHI))),
        //      -1.0 / GRAVITY, 0, 0;
        // Matrix3d Cnb = R * observation.transpose();
        Matrix3d Cnb = reference.transpose().inverse() * observation.transpose();
        this->Cbn = Cnb;
        // cout << "ref" << R << endl << endl;;
        cout << "obs" << observation.transpose() << endl << endl;;
        cout << "before" << endl << Cbn << endl << endl;;
        for(int i = 0; i < 30; ++i)
        {
            auto temp = (this->Cbn + (Cbn.transpose()).inverse()) * 0.5;
            this->Cbn = temp;
        }

        cout << "after" << endl << this->Cbn << endl << endl;
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
    Euler[0] = atan2(C(1, 0), C(0, 0));
    Euler[1] = atan2(-C(2, 0), sqrt(1 -  C(2, 0) * C(2, 0)));
    Euler[2] = atan2(C(2, 1), C(2, 2));

    for(int i = 0; i < 3; ++i)
        Euler[i] = Rad2Deg(Euler[i]);

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

bool Alignment::StaticAlignmentEpoch()
{
    if(OriginData[0].size() != OriginData[1].size())
        return false;

    Vector3d gn(0, 0, GRAVITY);
    Vector3d Omega_n(OMEGA * cos(Deg2Rad(PHI)), 0, -OMEGA * sin(Deg2Rad(PHI)));
    Vector3d vn = gn.cross(Omega_n);
    Matrix3d reference;
    reference.block(0, 0, 3, 1) = gn;
    reference.block(0, 1, 3, 1) = Omega_n;
    reference.block(0, 2, 3, 1) = vn;
    cout << reference << endl;
    // reference << -tan(PHI) / GRAVITY, 1.0 / (OMEGA * cos(PHI)), 0,
    //              0, 0, -1.0 / (GRAVITY * OMEGA * cos(PHI)),
    //              -1.0 / GRAVITY, 0, 0;
    for(int i = 0; i < this->OriginData[0].size(); ++i)
    {
        Vector3d gb = OriginData[0][i];
        Vector3d Omega_b = OriginData[1][i];
        Vector3d vb = gb.cross(Omega_b);
        Matrix3d observation;
        observation.block(0, 0, 3, 1) = gb;
        observation.block(0, 1, 3, 1) = Omega_b;
        observation.block(0, 2, 3, 1) = vb;

        try
        {
            Matrix3d Cnb = observation * reference.inverse();
            // cout << reference.inverse() << endl;
            // Matrix3d Cnb = reference * observation.transpose();
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
    }

    return true;
}

VecVector3d Alignment::GetEulerEpoch()
{
    return this->Euler_epoch;
}
