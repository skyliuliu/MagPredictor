#include "KalmanFilter.h"
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

KalmanFilter::KalmanFilter(int dim_x, int dim_z)
{
	/*矩阵初始化*/
    this->dim_x = dim_x;
    this->dim_z = dim_z;
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
    H = MatrixXd::Zero(dim_z, dim_x);
	
    K = MatrixXd::Zero(dim_x, dim_z);
    y = MatrixXd::Zero(dim_z, 1);
    S = MatrixXd::Zero(dim_z, dim_z);
    SI = MatrixXd::Zero(dim_z, dim_z);

    _I = MatrixXd::Identity(dim_x, dim_x);
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::predict()
{
	//x=Fx
    x << F * x;
	//P=FPF'+Q
    P << F * P * F.transpose() + Q;

	//save prior
    x_Prior = x;
    P_Prior = P;
}

void KalmanFilter::update(matXdArgConst& measurements)
{
    z = measurements;
	//y = z-Hx
    y << z - H * x; //error between measuremnet and prediction
	
	//common subexpression for speed
    MatrixXd PHT = P * H.transpose();

	//S = HPH' + R
	//project system uncertainty into measurement space
    S << H * PHT + R;
    //SI << S.inverse();
    SI = S.lu().inverse();

	//K =PH'inv(S)
	//map system uncertainty into kalman gain
    K = PHT * SI;

	//x = x+Ky
	//predict new x with residual scaled by the kalman gain
    x += K * y;

	// P = (I - KH)P(I - KH)' + KRK'
	// This is more numerically stable
	// and works for non - optimal K vs the equation
	// P = (I - KH)P usually seen in the literature.

    MatrixXd I_KH = _I - K * H;
    P << I_KH * P * I_KH.transpose() + K * R * K.transpose();

	//save measurement and posterior state
    x_Post = x;
    P_Post = P;
}

void KalmanFilter::clearAll()
{
    x.fill(0);
    x_Post = x;
    x_Prior = x;

    P = MatrixXd::Identity(dim_x, dim_x);
    P_Post = P;
    P_Prior = P;

    z = MatrixXd::Zero(dim_z, 1);
}
