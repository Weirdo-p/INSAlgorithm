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
        string line;
        stringstream buff;
        getline(ASCFile, line);
        replace(line.begin(), line.end(), ',', ' ');
        replace(line.begin(), line.end(), ';', ' ');
        buff << line;
        Vector3d gyro, accel;
        int week;
        double SOW;
        string comma, head;
        buff >> head >> comma; 
        cout << head << endl;
        break;
    }
}
