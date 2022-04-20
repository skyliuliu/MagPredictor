#ifndef MAGLOCALIZERPLUS_H
#define MAGLOCALIZERPLUS_H

#include <vector>
#include <iostream>
#include <QObject>
#include <QThread>
#include <QTimer>
#include <QDateTime>
#include <QDebug>

#include <Eigen/Dense>
#include "ukfom/ukf.hpp"
#include "ukfom/mtkwrap.hpp"

#include "mtk/build_manifold.hpp"
#include "mtk/types/vect.hpp"
#include "mtk/types/SOn.hpp"

#include "filtercommon.h"
#include "matrixcsv.h"
#include "magsensorparams.hpp"
//#include "si_motionctrol.h"
#include "KalmanFilter.h"
#include "MagLocalizerPro.h"

// We can't use types having a comma inside AutoConstruct macros :(
typedef MTK::vect<3, double> vec3;
typedef MTK::SO3<double> SO3;
/**
 * @brief MTK_BUILD_MANIFOLD
 * Magic, keep it!!
 */
MTK_BUILD_MANIFOLD (state ,
(( vec3 , pos ))
(( SO3 , orient ))
(( vec3, vel ))
)

struct myState:public state
{
    typedef myState self;
    self& operator=(const self& other)
    {
        pos = other.pos;
        orient = other.orient;
        vel = other.vel;
        return *this;
    }
};
typedef ukfom::mtkwrap<myState> capsuleState;
template<int dimOfMeasurement>
/**
 * @brief The MagEstimator class
 * 新版本的磁定位算法，用到了更为复杂的Manifold UKF算法
 */
class MagEstimator
{
private:
    ukfom::ukf<capsuleState> *kf;
    capsuleState cap_state;
    ukfom::ukf<capsuleState>::cov init_cov;
    static double dt;
    static double Q;
    static double R;
    static double capsuleMoment;
    static Eigen::MatrixXd SensorLoc;
    static MTK::vect<dimOfMeasurement, double> predictedData;
    static const int numOfBoards = dimOfMeasurement/3;
    static double scale;
    static myState ukf_fx(const myState &s)
    {
        //translate
        myState s_predict;
        s_predict.pos = s.pos + s.vel * dt;
        s_predict.orient = s.orient;
        s_predict.vel = s.vel;
        return s_predict;
    }
    static MTK::vect<dimOfMeasurement, double> ukf_hx(const myState &s)
    {
        //计算 measurement function
        MatrixXd B=MatrixXd::Zero(numOfBoards, 3);
        //"""计算Bx, By, Bz"""
        double x = s.pos(0, 0);
        double y = s.pos(1, 0);
        double z = s.pos(2, 0);

        double q0 = s.orient.w();
        double q1 = s.orient.x();
        double q2 = s.orient.y();
        double q3 = s.orient.z();

        double mx = 2*(-q0*q2+q1*q3);
        double my = 2*(q0*q1+q2*q3);
        double mz = (q0*q0 - q1*q1 - q2*q2 + q3*q3);

        MatrixXd pos = MatrixXd::Zero(numOfBoards, 3);
        pos.col(0).fill(x);
        pos.col(1).fill(y);
        pos.col(2).fill(z);
        pos -= SensorLoc;

        MatrixXd rotNorm = MatrixXd::Zero(numOfBoards,3);
        rotNorm.col(0).fill(mx);
        rotNorm.col(1).fill(my);
        rotNorm.col(2).fill(mz);

        MatrixXd rNorm = pos.rowwise().norm();
        MatrixXd r = MatrixXd::Zero(numOfBoards, 3);
        r.col(0) = rNorm.col(0);
        r.col(1) = rNorm.col(0);
        r.col(2) = rNorm.col(0);
        MatrixXd posNorm = pos.rowwise().normalized(); //9x3
        //线性叠加
        ArrayXXd r_3 = r.array().inverse().cube();
        ArrayXXd PosMInner = (posNorm.array()*rotNorm.array()).rowwise().sum();
        ArrayXXd PosMInnerEx = ArrayXXd::Zero(numOfBoards, 3);
        PosMInnerEx.col(0) = PosMInner.col(0);
        PosMInnerEx.col(1) = PosMInner.col(0);
        PosMInnerEx.col(2) = PosMInner.col(0);

        B = scale*capsuleMoment* (r_3*(3*PosMInnerEx*posNorm.array()-rotNorm.array())).matrix();

        //计算h(x)
        predictedData.middleRows(0, numOfBoards) = B.col(0);
        predictedData.middleRows(numOfBoards, numOfBoards) = B.col(1);
        predictedData.middleRows(2*numOfBoards, numOfBoards) = B.col(2);
        return predictedData; // Matrix（数据量， 1）
    }
    static ukfom::ukf<capsuleState>::cov ukf_Q()
    {
        ukfom::ukf<capsuleState>::cov cov = ukfom::ukf<capsuleState>::cov::Zero();
        cov.diagonal().setConstant(Q);
        return cov;
    }
    static MatrixXd ukf_R()
    {
        return R*MatrixXd::Identity(dimMeasurement, dimMeasurement);// R*I(81,81)
    }

public:
    typedef MTK::vect<dimOfMeasurement, double> BuildMeasurement;
    enum
    {
        DOF = myState::DOF, //状态量的自由度， 这里应为9
        dimState = 10, //3-pos, 4-quaternions, 3-velocity
        dimMeasurement = dimOfMeasurement // 输入测量的维度

    };

    /**
     * @brief MagEstimator
     * 初始化调参，影响初始收敛的效果
     *
     * @param params
     *
     */
    MagEstimator(MagSensorParams *params)
    {
        SensorLoc = params->getGeometryMatrix();
        dt = 0.03;

        //covariance 可调
        init_cov = 0.1 * ukfom::ukf<capsuleState>::cov::Identity();
        cap_state.pos << 10, 0, 5;//初始位置
        cap_state.vel << 0, 0, 0;//初始速度
        cap_state.orient.w() = 1;//初始四元数q0 = 1

        kf = new ukfom::ukf<capsuleState>(cap_state, init_cov);
    }

    void clearAll()
    {
        kf->muNoneConst().pos<<0, 0, 5;
        kf->muNoneConst().vel<<0,0,0;
        kf->muNoneConst().orient.w()=1;
        kf->muNoneConst().orient.x()=0;
        kf->muNoneConst().orient.y()=0;
        kf->muNoneConst().orient.z()=0;
        kf->sigmaNoneConst() = init_cov;
    }

    void estimate(const MTK::vect<dimMeasurement, double>& measurement)
    {
        kf->predict(ukf_fx, ukf_Q);
        kf->update(measurement, ukf_hx, ukf_R);
    }
    void setDipoleMoment(const double &moment)
    {
        capsuleMoment = moment;
    }
    void setTimeInterval(const double &dt)
    {
        this->dt = dt;
    }
    void setProcessNoise(const double &Q)
    {
        this->Q = Q;
    }
    void setMeasurementNoise(const double &R)
    {
        this->R = R;
    }
    int getDOF() const
    {
        return DOF;
    }
    double getTimeInterval()const
    {
        return dt;
    }

    Vector3d getVelocity()
    {
        Vector3d velocity;
        velocity(0) = kf->mu().vel(0,0);
        velocity(1) = kf->mu().vel(1,0);
        velocity(2) = kf->mu().vel(2,0);
        return velocity;
    }

    /**
     * @brief getPosAndRotVector
     * @return (7,1)matrix
     */
    MatrixXd getPosAndRotVector()
    {
        MatrixXd posRot = MatrixXd::Zero(7, 1);
        posRot(0, 0) = kf->mu().pos(0, 0);
        posRot(1, 0) = kf->mu().pos(1, 0);
        posRot(2, 0) = kf->mu().pos(2, 0);
        posRot(3, 0) = kf->mu().orient.w();
        posRot(4, 0) = kf->mu().orient.x();
        posRot(5, 0) = kf->mu().orient.y();
        posRot(6, 0) = kf->mu().orient.z();
        return posRot;
    }
    MatrixXd getPredictedData()
    {
        return predictedData;
    }
    MatrixXd getSensorLocationMatrix()
    {
        return SensorLoc;
    }
};

/**
 * @brief The MagLocalizerPlus class
 *  算法线程，用于接收数据和发送数据,
 *  并校准以及去除背景磁场
 */
typedef MagEstimator<MagSensorParams::NumOfFluxDataInTotal> capEstimator;
class MagLocalizerPlus : public QObject
{
    Q_OBJECT
public:
    explicit MagLocalizerPlus(QObject *parent = 0,
                              MagSensorParams *param = new MagSensorParams());
    ~MagLocalizerPlus();

private:
    void initSmoother();
    QThread *localizerThread;
    QTimer *localizerTimer;
    QTimer *caliTimer;
    capEstimator *predictor;
    KalmanFilter *smoother;

    capEstimator::BuildMeasurement rawMagData;
    capEstimator::BuildMeasurement filteredData;
    capEstimator::BuildMeasurement caliMagData;

    enum LocalizerStatus{
        started,
        calibrating,
        readyToStart,
        stopped
    }status;

    bool serialDataReady;
    bool bigMagDataReady;
    //calibrate variables & funcs
    int caliStatus;
    int maxCaliTimes;

    struct bigMag
    {
        MatrixXd location;
        float *angle;
        bool isInit;
        bool isXYZmove;
        bigMag()
        {
            location = MatrixXd::Zero(6, 2);
            angle = new float[2];
            isInit = false;
            isXYZmove = false;
        }
        void updateStatus(const float *loc, const float *angle)
        {
            location.col(0) = location.col(1);
            location(0, 1) = loc[0];
            location(1, 1) = loc[1];
            location(2, 1) = loc[2];
            location(3, 1) = loc[3];
            location(4, 1) = loc[4];
            location(5, 1) = loc[5];
            this->angle[0] = angle[0];
            this->angle[1] = angle[1];
            if(isInit == false)
            {
                isInit = true;
            }
            else
            {
                isXYZmove = ((abs(location(1,1) - location(1, 0))!=0)||
                            (abs(location(4,1) - location(4, 0))!=0)||
                            (abs(location(5,1) - location(5, 0))!=0))&&
                            (this->angle[0]==0 && this->angle[1]==0);
            }
        }
    }BigMagStatus;

signals:
    void sendFluxData(MatrixXd rawData, MatrixXd filteredData,
                      MatrixXd caliMagData, MatrixXd predictedData);
    void sendMagState(MatrixXd);
    void sendCaliStatus(int);

public slots:
    void init();
    void serialIn(MatrixXd rawMagData);
    void stop();

    void reCalibrate();
    void calibrateData();
    void updatePrediction();
    //void updateBigMagState(UAVObject *);

    void continuousStart();
};

#endif // MAGLOCALIZERPLUS_H
