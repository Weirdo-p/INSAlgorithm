#include "../include/mechanical.h"
#include "../include/common.h"
#include <iomanip>

const Vector3d omegaEIN(0.0, 0.0, OMEGA);

Mechanization::Mechanization(Euler pose, BLH prvBLH, ELLIPSOID type, Output prv) {
    this->prvPose = pose;
    this->prvPosi = prvBLH;
    this->type = type;
    this->prvVel = Vector3d(-6.360, -0.040, 0.049);
    // this->prvVel = Vector3d::Zero();
    this->prvOut = prv;
}

bool Mechanization::Update(Output now) {
    UpdateV(now);
    UpdatePosi(now);
    UpdatePose(now);
    this->prvOut = now;
    this->prvPose = this->nowPose;
    this->prvPosi = this->nowPosi;
    this->prvVel = this->nowVel;
    return true;
}

bool Mechanization::UpdatePosi(Output now) {
    this->nowPosi.H = prvPosi.H - 0.5 * (prvVel(2, 0) + nowVel(2, 0)) * 1.0 / ZCF;
    // cout << this->nowPosi.H << endl;

    double rm = RM(prvPosi.B, type);
    double h = 0.5 * (nowPosi.H + prvPosi.H);
    this->nowPosi.B = prvPosi.B + 0.5 * (prvVel(0, 0) + nowVel(0, 0)) / (rm + h) * 1.0 / ZCF;
    // cout << Rad2Deg(this->nowPosi.B) << endl;
    
    double phi = 0.5 * (prvPosi.B + nowPosi.B);
    double rn = RN(phi, type);
    // cout << setprecision(15) << rn << endl;
    this->nowPosi.L = prvPosi.L + 0.5 * (prvVel(1, 0) + nowVel(1, 0)) / ((rn + h) * cos(phi)) * 1.0 / ZCF;
    // cout << Rad2Deg(this->nowPosi.L) << endl;
}

bool Mechanization::UpdatePose(Output now) {
    // update body frame
    Vector3d deltaThetak(now.gx, now.gy, now.gz);
    Vector3d deltaThetakPrv(prvOut.gx, prvOut.gy, prvOut.gz);
    Vector3d phik = deltaThetak + 1.0 / 12.0 * deltaThetakPrv.cross(deltaThetak);
    Quaterniond phik_q = Vec2Qua(phik);
    // phik_q.normalize();

    BLH blh;
    blh.B = 0.5 * (prvPosi.B + nowPosi.B);
    blh.L = 0.5 * (prvPosi.L + nowPosi.L);
    blh.H = 0.5 * (prvPosi.H + nowPosi.H);
    double vn = (prvVel(0, 0) + nowVel(0, 0)) * 0.5;
    double ve = (prvVel(1, 0) + nowVel(1, 0)) * 0.5;
    Vector3d wnen = CalculateWnen(blh, ve, vn, type);
    // cout << wnen << endl << endl;
    Vector3d wein = CalculateWein(blh.B);
    // cout << wein << endl << endl;
    Vector3d ksai = (wnen + wein) * 1.0 / ZCF;
    Quaterniond ksai_q_T = Vec2Qua(ksai);
    // ksai_q_T.normalize();
    Quaterniond ksai_q = ksai_q_T.conjugate();
    ksai_q.conjugate();

    Matrix3d Cbn = Eu2Ro(this->prvPose);
    Quaterniond qbnPrv = Eu2Qu(this->prvPose);
    // qbnPrv.normalize();
    Quaterniond qbn = ksai_q * qbnPrv * phik_q;
    qbn.normalize();
    Matrix3d Cbn_now = qbn.matrix();

    Vector3d euler;
    CalculateEuler(Cbn_now, euler);
    this->nowPose.yaw = euler[0];
    this->nowPose.pitch = euler[1];
    this->nowPose.roll = euler[2];
}

bool Mechanization::UpdateV(Output now) {
    Vector3d vcor = Calculatevgcor(prvPosi);
    Vector3d ksai = Calculateksai(prvPosi);
    Vector3d vfkb = CalculateVfkb(now);
    Vector3d vfkn = Calculatevfkn(vfkb, ksai);
    this->nowVel = prvVel + vfkn + vcor;
}

Vector3d Mechanization::Calculatevgcor(BLH blh) {
    double g = GetGravity(type, prvPosi);
    Vector3d gl(0, 0, g);
    Vector3d wein = CalculateWein(blh.B);
    // cout << wein << endl;
    Vector3d wnen = CalculateWnen(blh, prvVel(1, 0), prvVel(0, 0), type);
    // 不太确定是不是代入上一个历元的速度
    Vector3d vcor = (gl - (2 * wein + wnen).cross(prvVel)) * 1.0 / ZCF;
    
    return vcor;
}

Vector3d Mechanization::Calculateksai(BLH blh) {
    Vector3d wein = CalculateWein(blh.B);
    Vector3d wnen = CalculateWnen(blh, prvVel(1, 0), prvVel(0, 0), type);
    Vector3d ksai = (wnen + wein) * 1.0 / ZCF;

    return ksai;
}

Vector3d Mechanization::CalculateVfkb(Output now) {
    Vector3d deltaVk(now.ax, now.ay, now.az);
    Vector3d deltaThetak(now.gx, now.gy, now.gz);

    Vector3d deltaVkPrv(prvOut.ax, prvOut.ay, prvOut.az);
    Vector3d deltaThetaPrv(prvOut.gx, prvOut.gy, prvOut.gz);

    Vector3d RotationCompensation = 0.5 * deltaThetak.cross(deltaVk);
    Vector3d ScullingCompensation = 1.0 / 12.0 * (deltaThetaPrv.cross(deltaVk) + deltaVkPrv.cross(deltaThetak));

    return (deltaVk + RotationCompensation + ScullingCompensation);
}

bool Mechanization::ZUPT(Output now) {
    if(abs(now.ax) > 2e-5) {
        return false;
    }
    this->nowVel = Vector3d::Zero();
    this->nowPosi = this->prvPosi;
    this->nowPose = this->prvPose;
    return true;
}

Vector3d Mechanization::Calculatevfkn(Vector3d vfkb, Vector3d ksai) {
    // Matrix3d k = ksai.matrix();
    // cout << vfkb << endl;
    Matrix3d I = Matrix3d::Identity();
    Matrix3d Cbn = Eu2Ro(this->prvPose);
    // cout << Cbn << endl;
    Matrix3d left = I - 0.5 * Vector2Matrix(ksai);
    Vector3d vfkn = left * Cbn * vfkb;

    return vfkn;
}


Vector3d Mechanization::CalculateWein(double phi) {
    return Vector3d(OMEGA * cos(phi), 0.0, -OMEGA * sin(phi));
}

Vector3d Mechanization::CalculateWnen(BLH blh, double ve, double vn, ELLIPSOID type) {
    double rn = RN(blh.B, type);
    double rm = RM(blh.B, type);
    double x = ve / (rn + blh.H);
    double y = - vn / (rm + blh.H);
    double z = - ve * tan(blh.B) / (rn + blh.H);
    // cout << x << "  " << y << "   " << z << endl;

    return Vector3d(x, y, z);
}

Vector3d Mechanization::GetNowVel() {
    return this->nowVel;
}

BLH Mechanization::GetNowPosi() {
    return this->nowPosi;
}

Euler Mechanization::GetNowEul() {
    return this->nowPose;
}
