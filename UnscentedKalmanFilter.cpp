#include "UnscentedKalmanFilter.h"
#include<Eigen/Cholesky>
#include<string>
#include<iostream>

using namespace std;

/*
 Sigma points method
*/
MerweScaledSigmaPoints::MerweScaledSigmaPoints(const int& dim_x, const float& alpha,
                                               const float& beta, const float& kappa)
{
    this->dim_x = dim_x;
    this->alpha = alpha;
    this->beta = beta;
    this->kappa = kappa;

    Wc = MatrixXd::Zero(2*dim_x + 1, 1);
    Wm = MatrixXd::Zero(2*dim_x + 1, 1);
    compute_weights();
}

MatrixXd MerweScaledSigmaPoints::sigma_points(matXdArgConst& x, matXdArgConst& P)
{
    /*
     * Computes the sigma points for an unscented Kalman filter
        given the mean (x) and covariance(P) of the filter.
        Returns tuple of the sigma points and weights.

        @Parameters
        ----------
        x: matrixXd (dim_x, 1)
        P: indentity(dim_x, dim_x)
    */
    const int n = dim_x;
    if (n != x.rows()){
        throw string("expected size(x) (%d,1), but the size is %d", n, x.rows());
    }
    double lambda_ = alpha*alpha*(n+kappa) - n;
    LLT<MatrixXd, Upper> lltOfP((lambda_+n)*P); //必须是upper，否则效果不好
    MatrixXd U = lltOfP.matrixL();
    MatrixXd sigmas = MatrixXd::Zero(2*n+1, n);
    MatrixXd xT = x.transpose();
    sigmas.row(0)=xT;
    for(int k=0;k<n;k++){
        sigmas.row(k+1) = xT + U.row(k);
        sigmas.row(n+k+1) = xT - U.row(k);
    }
    return sigmas;
}

void MerweScaledSigmaPoints::compute_weights()
{
    int n = dim_x;
    float lambda_ = alpha*alpha*(n+kappa) - n;
    float c = 0.5/(n+lambda_);
    Wc.fill(c);
    Wm.fill(c);
    Wc(0, 0) = lambda_/(n+lambda_) + (1- alpha*alpha + beta);
    Wm(0, 0) = lambda_/(n+lambda_);
}

/* UKF Definition */

UKF::UKF(const int& dim_x, const int& dim_z, const float& dt,
         MatrixXd (*fx)(matXdArgConst&, const float&),
         MatrixXd (*hx)(matXdArgConst&), MerweScaledSigmaPoints *points){
    _dim_x = dim_x;
    _dim_z = dim_z;
    x = MatrixXd::Zero(dim_x, 1);
    P = MatrixXd::Identity(dim_x, dim_x);
    x_Prior = x;
    x_Post = x;
    P_Prior = P;
    P_Post = P;

    Q = MatrixXd::Identity(dim_x, dim_x);
    R = MatrixXd::Identity(dim_z, dim_z);

    K = MatrixXd::Zero(dim_x, dim_z);
    y = MatrixXd::Zero(dim_z, 1);
    S = MatrixXd::Zero(dim_z, dim_z);
    SI = MatrixXd::Zero(dim_z, dim_z);
    eps = 0;
    //传入地址
    points_fn = points;
    this->fx = fx;
    this->hx = hx;
    _dt = dt;
    _num_sigmas = points->num_sigmas();
    Wc = points->Wc;
    Wm = points->Wm;
    sigmas_f = MatrixXd::Zero(_num_sigmas, _dim_x);
    sigmas_h = MatrixXd::Zero(_num_sigmas, _dim_z);
    predictUT = new UT(_num_sigmas);
    updateUT = new UT(_num_sigmas);
}

void UKF::predict(const float& dt){
    //calculate sigma points for given mean and covariance
    compute_process_sigmas(dt);
    //and pass sigmas through the unscented transform to compute prior
    predictUT->unscented_transform(sigmas_f, Wm, Wc, Q);
    x = predictUT->x;
    P = predictUT->P;

    x_Prior = x;
    P_Prior = P;
}

void UKF::update(matXdArgConst& measurements){
    // pass prior sigmas through h(x) to get measurement sigmas
    // the shape of sigmas_h will vary if the shape of z varies, so
    // recreate each time
    z = measurements;
    for(int i=0;i<_num_sigmas;i++){
        sigmas_h.row(i) = ((*hx)(sigmas_f.row(i).transpose())).transpose();
    }
    //mean and covariance of prediction passed through unscented transform
    updateUT->unscented_transform(sigmas_h, Wm, Wc, R);
    MatrixXd zp = updateUT->x;
    S = updateUT->P;
    //SI = S.inverse();
    SI = S.lu().inverse();//better for large matrix inverse

    //compute cross variance of the state and the measurements
    MatrixXd Pxz = cross_variance(x, zp, sigmas_f, sigmas_h);

    K = Pxz*SI; // Kalman gain
    y = z - zp;
    MatrixXd epsMat = y.transpose()*SI*y;
    eps = epsMat(0, 0);
    //update Gaussian state estimate (x, P)
    x = x + K*y;
    P = P - K*(S*K.transpose());

    x_Post = x;
    P_Post = P;

}

void UKF::compute_process_sigmas(const float& dt){
    //calculate sigma points for given mean and covariance
    MatrixXd sigmas = points_fn->sigma_points(x, P);
    for(int i=0;i<_num_sigmas;i++){
        sigmas_f.row(i) = (*fx)(sigmas.row(i), dt);
    }
}

MatrixXd cross(matXdArgConst& a, matXdArgConst& b){
    int dim_a = a.rows();
    int dim_b = b.rows();
    MatrixXd result = MatrixXd::Zero(dim_a, dim_b);
    MatrixXd bT = b.transpose();
    for(int i=0; i<dim_a;i++){
        result.row(i) = a(i, 0) * bT;
    }

    return result;
}

MatrixXd UKF::cross_variance(matXdArgConst& x, matXdArgConst& z,
                             matXdArgConst& sigmas_f, matXdArgConst& sigmas_h){
    //Compute cross variance of the state `x` and measurement `z`.
    MatrixXd Pxz = MatrixXd::Zero(sigmas_f.cols(), sigmas_h.cols());
    int N = sigmas_f.rows();
    for(int i=0;i<N;i++){
        MatrixXd dx = sigmas_f.row(i).transpose() - x;
        MatrixXd dz = sigmas_h.row(i).transpose() - z;
        Pxz += Wc(i, 0) * cross(dx, dz);
    }
    return Pxz;
}

void UKF::clearAll()
{
    x.fill(0);
    P = MatrixXd::Identity(_dim_x, _dim_x);
    x_Prior = x;
    x_Post = x;
    P_Prior = P;
    P_Post = P;
    sigmas_f.fill(0);
    sigmas_h.fill(0);
    K = MatrixXd::Zero(_dim_x, _dim_z);
    y = MatrixXd::Zero(_dim_z, 1);
    S = MatrixXd::Zero(_dim_z, _dim_z);
    SI = MatrixXd::Zero(_dim_z, _dim_z);
}
