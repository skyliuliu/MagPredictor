#ifndef FILTERUTILS
#define FILTERUTILS
#include "ukfom/ukf.hpp"
#include "ukfom/mtkwrap.hpp"
#include "mtk/build_manifold.hpp"
#include "mtk/types/vect.hpp"
#include "mtk/types/pose.hpp"
#include "mtk/types/SOn.hpp"
#include "KalmanFilter.h"

typedef MTK::vect<1, double> vec1;
typedef MTK::vect<3, double> vec3;
typedef MTK::vect<4, double> vec4;
typedef MTK::SO3<double> SO3;

template<int dimension>
class Differentator
{
public:
    explicit Differentator(double noise = 1, double dt=0.03)
    {
        kf = new KalmanFilter(2*dimension, dimension);
        kf->H = MatrixXd::Zero(dimension, 2*dimension);
        for(int i=0;i<dimension;i+=2){
            kf->x(i, 0) = 0.0;
        }
        for(int i=0;i<dimension;i++){
            kf->F(2*i, 1+2*i) = dt;
            kf->H(i, 2*i) = 1;
        }
        kf->P *= 50;
        kf->R *= noise;
        kf->Q *= 0.1;
    }

    /**
     * @brief operator ()
     * @param fx
     * @return 返回kalman filter预测的微分
     */
    MatrixXd diff(matXdArgConst& fx)
    {
        kf->predict();
        kf->update(fx);
        Map<MatrixXd> result(kf->x.data(), 2, dimension);
        return result.row(1).transpose();
    }

    MatrixXd filter(matXdArgConst& fx)
    {
        kf->predict();
        kf->update(fx);
        Map<MatrixXd> result(kf->x.data(), 2, dimension);
        return result.row(0).transpose();
    }
private:
   KalmanFilter *kf;
};

namespace RotRep {

static const double PI =3.14159265359;

inline double sind(const double& degree)
{
    return sin(degree*PI/180);
}

inline double cosd(const double& degree)
{
    return cos(degree*PI/180);
}

struct xyzCoordinate
{
    double x;
    double y;
    double z;

};

struct thetaAlphaCoordinate
{
    double theta;
    double alpha;

    void fromXYZ(double x, double y, double z)
    {
        double norm = sqrt(x*x + y*y + z*z);
        theta = acos(z/norm);
        if(theta != 0)
        {
            alpha = atan2(x, y);
        }
        else alpha = 0;
    }

    double degTheta()
    {
        return theta*180/PI;
    }

    double degAlpha()
    {
        return alpha*180/PI;
    }

    thetaAlphaCoordinate()
    {
        theta =0;
        alpha =0;
    }
};
}
#endif // FILTERUTILS

