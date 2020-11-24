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
    /*****************************
     * function: user interface
     * @param   now
     * @return  true if success
    *****************************/
    bool      Update(Output now);

private:
    /**************************
     * ZUPt
     * @param now   output
     * @return  true if static
    **************************/
    bool      ZUPT(Output now);

    /***************************
     * function: update velocity
     * @param now
     * @return  true if success
    ***************************/
    bool      UpdateV(Output now);

    /*****************************
     * function: update position
     * @param   now 
     * @return  true if success
    *****************************/
    bool      UpdatePosi(Output now);

    /********************************
     * update pose
     * @param   now
     * @return  true if success
    ********************************/
    bool      UpdatePose(Output now);

// helpers, part of velocity updating
private:
    Vector3d  Calculatevgcor(BLH blh);

    Vector3d  Calculateksai(BLH blh);

    Vector3d  CalculateVfkb(Output now);

    Vector3d  Calculatevfkn(Vector3d vfkb, Vector3d ksai);

private:
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