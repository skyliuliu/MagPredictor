#pragma once
#include <iostream>
#include <Eigen/Dense>
#include "filtercommon.h"
using namespace std;
using namespace Eigen;

class KalmanFilter
{
public:
	KalmanFilter(int dim_x, int dim_z); //构造函数
	~KalmanFilter();
	void predict();
    void update(matXdArgConst& measurements);
    void clearAll();
	
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
	MatrixXd H; //measurement model

	MatrixXd K; //Kalman Gain
	MatrixXd y; //Residual 
	MatrixXd S; //system uncertainty
	MatrixXd SI;//inverse system uncertainty

private:
	MatrixXd _I; //identity matrix. Do not alter this.
    int dim_x;
    int dim_z;
};

