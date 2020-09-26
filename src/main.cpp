#include <iostream>
#include <eigen3/Eigen/Core>
#include "../include/common.h"
#include "../include/calibrateAccel.h"
#include "../include/calibrateGyro.h"

using namespace std;
using namespace Eigen;

int main()
{
    string Accel[6];
    Accel[0] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Static/XUp.ASC";
    Accel[1] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Static/XDown.ASC";
    Accel[2] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Static/YUp.ASC";
    Accel[3] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Static/YDown.ASC";
    Accel[4] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Static/ZUp.ASC";
    Accel[5] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Static/ZDown.ASC";
    CalibrateAccel AccelCalibrator(Accel);
    AccelCalibrator.CalculateM();
    VecVector3d* origin = AccelCalibrator.GetAccelOrigin();
    // for(auto data : origin[1])
    // {
    //     cout << data << endl << endl;
    // }
    Matrix34d M = AccelCalibrator.GetM();
    cout << "Accel------M\n" << M << endl << endl;
    VecVector3d compensated;
    Compensate(M, origin[0], compensated);
    // for(auto data : compensated)
    // {
    //     cout << data << endl << endl;
    // }

    string Gyro[6];
    Gyro[0] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Kinematic/X-negative360.ASC";
    Gyro[1] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Kinematic/X-positive360.ASC";
    Gyro[2] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Kinematic/Y-positive360.ASC";
    Gyro[3] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Kinematic/Y-negative360.ASC";
    Gyro[4] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Kinematic/Z-negative360.ASC";
    Gyro[5] = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Kinematic/Z-positive360.ASC";
    CalibrateGyro GyroCalibrator(Gyro, Accel);
    Matrix34d M1;
    GyroCalibrator.CalculateM();
    M1 = GyroCalibrator.GetM();
    cout << M1 << endl << endl;
    VecVector3d* origin_K = GyroCalibrator.GetOrigin();
    VecVector3d gyro_compensated;
    Compensate(M1, origin_K[2], gyro_compensated);

    // for(auto data : gyro_compensated)
    // {
    //     cout << data << endl << endl;
    // }
    return 0;
}