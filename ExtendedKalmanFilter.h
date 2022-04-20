#ifndef EKF_H
#define EKF_H
#include <iostream>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

class EKF
{
public:
    EKF(int dim_x, int dim_z);
    ~EKF();
    void predict();
    void predict_x();
    void predict_P();
    void update(MatrixXd measurements);
    void updateHJacob(MatrixXd jacob);
    void updateHx(MatrixXd hx);
    /*
    Kalman Filter 矩阵设置
    */
    MatrixXd z; // measurement
    MatrixXd x; // KF state
    MatrixXd x_Post;
    MatrixXd x_Prior;

    MatrixXd P; // covariance
    MatrixXd P_Post; // post covariance
    MatrixXd P_Prior; // Prior covariance

    MatrixXd R; // measurement noise
    MatrixXd Q; // process noise

    MatrixXd F; //prediction model
    MatrixXd FJacob; // prediction F jacobian

    MatrixXd K; //Kalman Gain
    MatrixXd y; //Residual
    MatrixXd S; //system uncertainty
    MatrixXd SI;//inverse system uncertainty
private:
    MatrixXd _I; //identity matrix. Do not alter this.
    MatrixXd HJacob; //Jacobians Matrix
    MatrixXd Hx; // measurement function
};

#endif // EKF_H
