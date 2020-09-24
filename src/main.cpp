#include <iostream>
#include <Eigen/Core>
#include "../include/common.h"

using namespace std;
using namespace Eigen;

int main()
{
    string ASCtest = "/home/weirdo/Documents/coding/INSAlgorithm/data/firstgroup/Kinematic/X-negative360.ASC";
    VecVector3d Accel, Gyro;
    ReadASC(ASCtest, Accel, Gyro);
    Matrix<double, 3, 3> test;
    test << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    cout << test << endl;
    cout << "test" << endl;
    return 0;
}