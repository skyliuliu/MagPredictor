#ifndef FIXLAGSMOOTHER_H
#define FIXLAGSMOOTHER_H

#include "KalmanFilter.h"
#include "filtercommon.h"

class FixLagSmoother
{
public:
    FixLagSmoother(const int dim_x, const int dim_z, const int N);
    void smooth(matXdArgConst& measurements);

public:
    /*
    FixLagSmoother 矩阵设置
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
    MatrixXd H; //measurement model

    MatrixXd K; //Kalman Gain
    MatrixXd y; //Residual
    MatrixXd S; //system uncertainty
    MatrixXd SI;//inverse system uncertainty

    std::vector<MatrixXd> xSmooth;//vector容器用于存储多个matrix


private:
    int n;
    MatrixXd _I; //identity matrix. Do not alter this.
    int count;
};

#endif // FIXLAGSMOOTHER_H
