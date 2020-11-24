#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Core>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <string.h>
#include "../include/common.h"
#include "../include/calibrateAccel.h"
#include "../include/calibrateGyro.h"
#include "../include/initialalignment.h"
#include "../include/mechanical.h"

using namespace std;
using namespace Eigen;

int main()
{
    // 以下为机械编排算法
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    // char binPath[] = "/home/weirdo/Documents/code/dataset/立德A15跑车原始数据/D/A15_imu.bin";
    char binPath[] = "/home/weirdo/Documents/code/C++/INSAlgorithm/data/mechanical/Data1.bin";
    FILE* file = fopen(binPath, "rb");
    if(file == NULL)
    {
        cout << "can not open file, please check!" << endl;
        return 1;
    }

    // forward lide data
    // double euler[3];
    // euler[2] = Deg2Rad(-0.00137516);
    // euler[1] = Deg2Rad(-0.43093389);
    // euler[0] = Deg2Rad(180.38599896);
    // Euler init(euler[2], euler[1], euler[0]);
    // BLH initPosi(Deg2Rad(30.4445199466), Deg2Rad(114.4718777663), 21.016);
    // ELLIPSOID type(a_g, b_g);
    ////////////////////////////////////////////////////////////////////////////
    // 算法验证
    double euler[3];
    euler[2] = Deg2Rad(0.0107951084511778);
    euler[1] = Deg2Rad(-2.14251290749072);
    euler[0] = Deg2Rad(-75.7498049214083);
    Euler init(euler[2], euler[1], euler[0]);
    BLH initPosi(Deg2Rad(23.1373950708), Deg2Rad(113.3713651222), 2.175);
    ELLIPSOID type(a_g, b_g);

    Output prv;
    while(!feof(file)) {
        GetOutput(file, prv);
        if(prv.time < 91620)
            continue;
        break;
    }
    Mechanization solver(init, initPosi, type, prv);

    int count = 0;
    double tmp;
    ofstream out("./test_data.txt", ios::app);
    while(!feof(file)) {
        count ++;
        Output now;
        GetOutput(file, now);

        solver.Update(now);
        Vector3d vel = solver.GetNowVel();
        BLH posi = solver.GetNowPosi();
        Euler eu = solver.GetNowEul();
        out << fixed << setprecision(3) << right;
        out << now.time << " ";
        out << setprecision(15);
        out << Rad2Deg(posi.B) << " " << Rad2Deg(posi.L) << " " << posi.H << " ";
        out << vel[0] << " " << vel[1] << " " << vel[2] << " ";
        out << Rad2Deg(eu.roll) << " " << Rad2Deg(eu.pitch) << " " << Rad2Deg(eu.yaw) << endl;
    }
}

