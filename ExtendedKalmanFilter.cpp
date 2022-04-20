#include "ExtendedKalmanFilter.h"
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

EKF::EKF(int dim_x, int dim_z)
{
    //矩阵初始化
    z = MatrixXd::Zero(dim_z, 1);

    x = MatrixXd::Zero(dim_x, 1);
    x_Post = x;
    x_Prior = x;

    P = MatrixXd::Identity(dim_x, dim_x);
    P_Post = P;
    P_Prior = P;

    R = MatrixXd::Identity(dim_z, dim_z);
    Q = MatrixXd::Identity(dim_x, dim_x);

    F = MatrixXd::Identity(dim_x, dim_x);
    FJacob = MatrixXd::Identity(dim_x, dim_x);
    HJacob = MatrixXd::Zero(dim_z, dim_x);
    Hx = z;

    K = MatrixXd::Zero(dim_x, dim_z);
    y = MatrixXd::Zero(dim_z, 1);
    S = MatrixXd::Zero(dim_z, dim_z);
    SI = MatrixXd::Zero(dim_z, dim_z);

    _I = MatrixXd::Identity(dim_x, dim_x);
}

EKF::~EKF(){

}

void EKF::predict(){
    //x=Fx
    x << F * x;
    //P=FPF'+Q
    P << FJacob * P * FJacob.transpose() + Q;

    //save prior
    x_Prior = x;
    P_Prior = P;
}

void EKF::predict_x(){
    //x=Fx
    x << F * x;
    x_Prior = x;
}

void EKF::predict_P(){
    //P=FPF'+Q
    P << FJacob * P * FJacob.transpose() + Q;
    P_Prior = P;
}

void EKF::update(MatrixXd measurements){

//    HJacob: Jacobian Matrix function (dim_z, dim_x)
//    Hx: h(x) measurement function (dim_z,1)

    z = measurements;
    //y = z-h(x)
    y << z - Hx; //error between measuremnet and prediction

    //common subexpression for speed
    MatrixXd PHT = P * (HJacob.transpose());

    //S = HPH' + R
    //project system uncertainty into measurement space
    S << HJacob * PHT + R;
    //SI << S.inverse();
    SI = S.lu().inverse();

    //K =PH'inv(S)
    //map system uncertainty into kalman gain
    K = PHT * SI;

    //x = x+Ky
    //predict new x with residual scaled by the kalman gain
    x << x + K * y;

    // P = (I - KH)P(I - KH)' + KRK'
    // This is more numerically stable
    // and works for non - optimal K vs the equation
    // P = (I - KH)P usually seen in the literature.

    MatrixXd I_KH = _I - K * HJacob;
    P << I_KH * P * I_KH.transpose() + K * R * K.transpose();

    //save measurement and posterior state
    x_Post = x;
    P_Post = P;
}

void EKF::updateHJacob(MatrixXd jacob)
{
    HJacob = jacob;
}

void EKF::updateHx(MatrixXd hx)
{
    Hx = hx;
}
