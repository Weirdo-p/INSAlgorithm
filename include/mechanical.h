#ifndef _MECHANICAL_H_
#define _MECHANICAL_H_

#include "common.h"
#include <stdio.h>


class Mechanization
{
/* constructor */
public:
    Mechanization() {  }

    Mechanization(Euler pose, BLH prvBLH, ELLIPSOID type, Output prv);

public:
    /***********************************
     * function:     姿态更新
     * @param fp  [in]  file point
     * @param prv [in]  previous pose
     * @param mow [out] now pose
    ***********************************/
    // bool UpdatePose(FILE* fp, Euler prv, BLH prvBLH, Euler &now);

    bool      UpdateV(Output now);

    bool      UpdatePosi(Output now);

    bool      UpdatePose(Output now);

    bool      Update(Output now);

    Vector3d  Calculatevgcor(BLH blh);

    Vector3d  Calculateksai(BLH blh);

    Vector3d  CalculateVfkb(Output now);

    Vector3d  Calculatevfkn(Vector3d vfkb, Vector3d ksai);

    Vector3d  CalculateWein(double phi);

    Vector3d  CalculateWnen(BLH blh, double ve, double vn, ELLIPSOID type);
public:
    Vector3d  GetNowVel();

    Euler     GetNowEul();
    
    BLH       GetNowPosi();

private:
    ELLIPSOID type;
    Euler     prvPose;
    Euler     nowPose;

    BLH       prvPosi;
    BLH       nowPosi;

    Vector3d  prvVel;
    Vector3d  nowVel;

    Output    prvOut;
    Output    nowOut;
};

#endif