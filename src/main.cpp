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
    // 以下为初始对准  标定算法
    //////////////////////////////////////////////////////////////////////////////////////////
    // string Accel[6];
    // Accel[0] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Static/XUp.ASC";
    // Accel[1] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Static/XDown.ASC";
    // Accel[2] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Static/YUp.ASC";
    // Accel[3] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Static/YDown.ASC";
    // Accel[4] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Static/ZUp.ASC";
    // Accel[5] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Static/ZDown.ASC";
    // CalibrateAccel AccelCalibrator(Accel);
    // AccelCalibrator.CalculateM();
    // VecVector3d* origin = AccelCalibrator.GetAccelOrigin();

    // Matrix34d M = AccelCalibrator.GetM();
    // cout << "Accel-----------------\n" << M << endl << endl;
    // VecVector3d compensated;
    // Compensate(M, origin[5], compensated);

    // string Gyro[6];
    // Gyro[0] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Kinematic/X-negative360.ASC";
    // Gyro[1] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Kinematic/X-positive360.ASC";
    // Gyro[2] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Kinematic/Y-negative360.ASC";
    // Gyro[3] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Kinematic/Y-positive360.ASC";
    // Gyro[4] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Kinematic/Z-negative360.ASC";
    // Gyro[5] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Kinematic/Z-positive360.ASC";
    // CalibrateGyro GyroCalibrator(Gyro, Accel);
    // Matrix34d M1;
    // GyroCalibrator.CalculateM();
    // M1 = GyroCalibrator.GetM();
    // VecVector3d* origin_K = GyroCalibrator.GetOrigin_Static();
    // VecVector3d gyro_compensated;
    // Compensate(M1, origin_K[0], gyro_compensated);
    // cout << "Gyro------------------" << endl;
    // cout << M1 << endl << endl;


    // string LINSPath = "/home/weirdo/Documents/coding/INSAlgorithm/data/data3.txt";
    // VecVector3d LinsAccel, LinsGyro;
    // ReadLinsData(LINSPath, LinsAccel, LinsGyro);
    // Alignment alignment(LinsGyro, LinsAccel);

    // alignment.StaticAlignmentMean();
    // double* euler = alignment.GetEulerMean();
    //////////////////////////////////////////////////////////////////////////////////////////////////////

    // 以下为机械编排算法
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    char binPath[] = "/home/weirdo/Documents/coding/INSAlgorithm/data/mechanical/Data1.bin";
    FILE* file = fopen(binPath, "rb");
    if(file == NULL)
    {
        cout << "can not open file, please check!" << endl;
        return 1;
    }
    // VecVector3d accel, gyro;
    // DetectStaticData(file, gyro, accel);
    // Alignment align(gyro, accel);
    // align.StaticAlignmentMean();
    // double* euler = align.GetEulerMean();
    // for(int i = 0; i < 3; ++ i)
    //     cout << Rad2Deg(euler[i]) << endl;

    double euler[3];
    euler[2] = Deg2Rad(-75.7498049314083);
    euler[1] = Deg2Rad(-2.14251290749072);
    euler[0] = Deg2Rad(0.0107951084511778);
    Euler init(euler[0], euler[1], euler[2]);
    BLH initPosi(Deg2Rad(23.1373950708), Deg2Rad(113.3713651222), 2.175);
    ELLIPSOID type(a_g, b_g);
    
    Output prv;
    while(!feof(file)) {
        GetOutput(file, prv);
        // cout << setprecision(15) << now.time << endl;
        if(prv.time < 91620)
            continue;
        break;
    }
    Mechanization solver(init, initPosi, type, prv);

    int count = 0;
    ofstream out("/home/weirdo/Documents/coding/INSAlgorithm/result/data1.txt", ios::app);
    while(!feof(file)) {
        Output now;
        GetOutput(file, now);
        // cout << setprecision(15) << now.time << endl;
        if(now.time < 91620)
            continue;

        solver.Update(now);
        Vector3d vel = solver.GetNowVel();
        BLH posi = solver.GetNowPosi();
        Euler eu = solver.GetNowEul();
        out << fixed << setprecision(9) << right;
        out << now.time << " ";
        out << Rad2Deg(posi.B) << " " << Rad2Deg(posi.L) << " " << posi.H << " ";
        out << vel[0] << " " << vel[1] << " " << vel[2] << " ";
        out << Rad2Deg(eu.roll) << " " << Rad2Deg(eu.pitch) << " " << Rad2Deg(eu.yaw) << endl;
        // count ++;
        // if(count >= 400)
        //     break;
    }

    // struct temp
    // {
    //     double t, lat, lon, height;
    //     double vn, ve, vd;
    //     double roll, pitch, yaw;
    // };
    
    // FILE* fp = fopen("/home/weirdo/Documents/coding/INSAlgorithm/data/mechanical/Data1_PureINS.bin", "rb");
    // ofstream out("/home/weirdo/Documents/coding/INSAlgorithm/data/mechanical/Data1_PureINS.txt", ios::binary | ios::app);
    // while (!feof(fp))
    // {
    //     temp tp;
    //     fread(&tp, sizeof(tp), 1, fp);
    //     out << setprecision(15) << tp.t << "," << tp.lat << "," << tp.lon << "," << tp.height << "," << tp.vn << "," << tp.ve << "," << tp.vd << "," << tp.roll << "," << tp.pitch << "," << tp.yaw << endl;
    // }
    

}

