#ifndef UKF_H
#define UKF_H
#include<Eigen/Dense>
#include"filtercommon.h"
using namespace Eigen;

class MerweScaledSigmaPoints{
    /*
    Generates sigma points and weights according to Van der Merwe's
    2004 dissertation[1] for the UnscentedKalmanFilter class.. It
    parametizes the sigma points using alpha, beta, kappa terms, and
    is the version seen in most publications.
    Unless you know better, this should be your default choice.
    */
public:
    MerweScaledSigmaPoints(const int& dim_x, const float& alpha,
                           const float& beta, const float& kappa);
    int num_sigmas()const{return 2*dim_x+1;}
    MatrixXd sigma_points(matXdArgConst& x, matXdArgConst& P);
    MatrixXd Wc;
    MatrixXd Wm;

private:
    int dim_x;
    float alpha;
    float beta;
    float kappa;
    void compute_weights();
};

class UKF
{
public:
    UKF(const int& dim_x, const int& dim_z, const float& dt,
        MatrixXd (*fx)(matXdArgConst&, const float&), MatrixXd (*hx)(matXdArgConst&),
        MerweScaledSigmaPoints *points);
    void predict(const float& dt);
    void update(matXdArgConst& measurements);
    void clearAll();
    MatrixXd cross_variance(matXdArgConst& x, matXdArgConst& z,
                            matXdArgConst& sigmas_f, matXdArgConst& sigmas_h);
    void compute_process_sigmas(const float& dt);
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


    MatrixXd K; //Kalman Gain
    MatrixXd y; //Residual
    MatrixXd S; //system uncertainty
    MatrixXd SI;//inverse system uncertainty
    double eps;

    MatrixXd (*fx)(matXdArgConst&, const float&) ; //fx function
    MatrixXd (*hx)(matXdArgConst&) ; //hx function

    MerweScaledSigmaPoints *points_fn;

private:
    int _dim_x;
    int _dim_z;
    float _dt;
    int _num_sigmas;
    MatrixXd Wc;
    MatrixXd Wm;
    //sigma points transformed through f(x) and h(x)
    //variables for efficiency so we don't recreate every update
    MatrixXd sigmas_f;
    MatrixXd sigmas_h;
    struct UT{
        MatrixXd x;
        MatrixXd P;
        int numOfSigmas;
        UT(const int & numOfSigmas)
        {
            this->numOfSigmas = numOfSigmas;
        }
        void unscented_transform(matXdArgConst& sigmas, matXdArgConst& Wm,
                                 matXdArgConst& Wc,
                                 matXdArgConst& noise_cov){
            MatrixXd x = Wm.transpose()*sigmas;
            MatrixXd y = sigmas;
            for(int i=0;i<numOfSigmas;i++){
                y.row(i) -= x;
            }
            P = y.transpose() * (Wc.asDiagonal()*y);
            P += noise_cov;
            this->x = x.transpose();
        }
    };
    UT *predictUT;
    UT *updateUT;
};

#endif // UKF_H
