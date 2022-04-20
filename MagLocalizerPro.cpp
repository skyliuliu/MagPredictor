#include "MagLocalizerPro.h"

//static 变量定义
MatrixXd MagLocalizerPro::sensorLoc;
MatrixXd MagLocalizerPro::rawFlux;
MatrixXd MagLocalizerPro::deltaRawFlux;
MatrixXd MagLocalizerPro::deltaEstimateFlux;
double MagLocalizerPro::bigMagMoment = 1850;
double MagLocalizerPro::capMoment = 0.29;
double MagLocalizerPro::dt = 0.03;
Vector3d MagLocalizerPro::dpos_bigMag;
Vector3d MagLocalizerPro::pos_bigMag;

MatrixXd MagLocalizerPro::createFlux(double x, double y, double z,
                                      double m,
                                      double mx, double my, double mz,
                                      matXdArgConst&sensorLoc)
{
    //计算 measurement function
    int numOfBoards = MagSensorParams::NumOfSensors;
    MatrixXd B=MatrixXd::Zero(numOfBoards, 3);
    //"""计算Bx, By, Bz"""
    //限制范围
    MatrixXd pos = MatrixXd::Zero(numOfBoards, 3);
    pos.col(0).fill(x);
    pos.col(1).fill(y);
    pos.col(2).fill(z);
    pos -= sensorLoc;

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

    B = 1e6*m* (r_3*(3*PosMInnerEx*posNorm.array()-rotNorm.array())).matrix();
    Map<MatrixXd> Breshape(B.data(), MagSensorParams::NumOfFluxDataInTotal, 1);
    return Breshape;
}

magState MagLocalizerPro::ukf_fx(const magState &s)
{
    magState s_predict;
    s_predict.bigMagPos = s.bigMagPos + s.bigMagVel * dt;
    s_predict.bigMagOrient = s.bigMagOrient;
    s_predict.bigMagVel = s.bigMagVel;
    s_predict.bigMagMoment = s.bigMagMoment;
    //capsule
    s_predict.capPos  =s.capPos + s.capVel * dt;
    s_predict.capDeltaPos = s.capDeltaPos;
    s_predict.capOrient = s.capOrient;
    //s_predict.capDeltaOrient = s.capDeltaOrient;

    s_predict.capVel = s.capVel;
    return s_predict;
}

/**
 * @brief MagLocalizerPro::ukf_hx
 * 计算微分方程
 * @param s state
 * @return
 */
MagLocalizerPro::BuildMeasurement MagLocalizerPro::ukf_hx(const magState &s)
{
    int numOfBoards = MagSensorParams::NumOfSensors;
    int numOfTotalData = MagSensorParams::NumOfFluxDataInTotal;

    //计算 measurement function
    //big Mag
    double b_moment = s.bigMagMoment(0, 0);
    double b_x = s.bigMagPos(0, 0);
    double b_y = s.bigMagPos(1, 0);
    double b_z = s.bigMagPos(2, 0);
    double b_q0 = s.bigMagOrient.w();
    double b_q1 = s.bigMagOrient.x();
    double b_q2 = s.bigMagOrient.y();
    double b_q3 = s.bigMagOrient.z();
    double b_mx = 2*(-b_q0*b_q2+b_q1*b_q3);
    double b_my = 2*(b_q0*b_q1+b_q2*b_q3);
    double b_mz = b_q0*b_q0 - b_q1*b_q1 - b_q2*b_q2 + b_q3*b_q3;
    MatrixXd b_pos = MatrixXd::Zero(numOfBoards, 3);
    b_pos.fill(b_x);
    b_pos.fill(b_y);
    b_pos.fill(b_z);
    b_pos -= sensorLoc;
    MatrixXd b_a = b_pos.col(0);
    MatrixXd b_b = b_pos.col(1);
    MatrixXd b_c = b_pos.col(2);
    MatrixXd b_a2 = b_pos.col(0).array().pow(2);
    MatrixXd b_b2 = b_pos.col(1).array().pow(2);
    MatrixXd b_c2 = b_pos.col(2).array().pow(2);

    MatrixXd b_r2 = b_pos.rowwise().norm().array().pow(2);
    MatrixXd b_r7 = b_pos.rowwise().norm().array().pow(7);

    MatrixXd b_posDotM = b_mx*b_a + b_my*b_b + b_mz*b_c;
    MatrixXd b_amx = b_mx*b_a;
    MatrixXd b_bmy = b_my*b_b;
    MatrixXd b_cmz = b_mz*b_c;
    MatrixXd b_mxR2 = b_mx*b_r2;
    MatrixXd b_myR2 = b_my*b_r2;
    MatrixXd b_mzR2 = b_mz*b_r2;

    //capsule
    double c_moment = capMoment;
    double c_x = s.capPos(0, 0);
    double c_y = s.capPos(1, 0);
    double c_z = s.capPos(2, 0);
    double c_q0 = s.capOrient.w();
    double c_q1 = s.capOrient.x();
    double c_q2 = s.capOrient.y();
    double c_q3 = s.capOrient.z();
    double c_mx = 2*(-c_q0*c_q2+c_q1*c_q3);
    double c_my = 2*(c_q0*c_q1+c_q2*c_q3);
    double c_mz = c_q0*c_q0 - c_q1*c_q1 - c_q2*c_q2 + c_q3*c_q3;
    MatrixXd c_pos = MatrixXd::Zero(numOfBoards, 3);
    c_pos.fill(c_x);
    c_pos.fill(c_y);
    c_pos.fill(c_z);
    c_pos -= sensorLoc;
    MatrixXd c_a = c_pos.col(0);
    MatrixXd c_b = c_pos.col(1);
    MatrixXd c_c = c_pos.col(2);
    MatrixXd c_a2 = c_pos.col(0).array().pow(2);
    MatrixXd c_b2 = c_pos.col(1).array().pow(2);
    MatrixXd c_c2 = c_pos.col(2).array().pow(2);

    MatrixXd c_r2 = c_pos.rowwise().norm().array().pow(2);
    MatrixXd c_r7 = c_pos.rowwise().norm().array().pow(7);

    MatrixXd c_posDotM = c_mx*c_a + c_my*c_b + c_mz*c_c;
    MatrixXd c_amx = c_mx*c_a;
    MatrixXd c_bmy = c_my*c_b;
    MatrixXd c_cmz = c_mz*c_c;
    MatrixXd c_mxR2 = c_mx*c_r2;
    MatrixXd c_myR2 = c_my*c_r2;
    MatrixXd c_mzR2 = c_mz*c_r2;

    MatrixXd _aq2bq1cq0 = -c_a*c_q0+c_b*c_q1+c_c*c_q0;
    MatrixXd aq3bq0_cq1 = c_a*c_q3+c_b*c_q0-c_c*c_q1;
    MatrixXd aq0_bq3cq2 = c_a*c_q0-c_b*c_q3+c_c*c_q2;
    MatrixXd aq1bq2cq3 = c_a*c_q1+c_b*c_q2+c_c*c_q3;

    MatrixXd deltaFlux = MatrixXd::Zero(numOfBoards, 3);
    deltaFlux.col(0) = -3*b_moment*scale*(dpos_bigMag(0)*(2*b_a2.cwiseProduct(b_posDotM)
                                            + b_a.cwiseProduct(3*b_a.cwiseProduct(b_posDotM) - b_mxR2)
                                            - b_r2.cwiseProduct(2*b_amx+b_bmy+b_cmz)
                                            )
                                        +dpos_bigMag(1)*(b_a.cwiseProduct(2*b_b.cwiseProduct(b_posDotM) - b_myR2)
                                             + b_b.cwiseProduct(3*b_a.cwiseProduct(b_posDotM) - b_mxR2)
                                             )
                                        +dpos_bigMag(2)*(b_a.cwiseProduct(2*b_c.cwiseProduct(b_posDotM) - b_mzR2)
                                             + b_c.cwiseProduct(3*b_a.cwiseProduct(b_posDotM) - b_mxR2)
                                             )
                                        ).cwiseQuotient(b_r7)
                       +c_moment*scale*(-3*(s.capDeltaPos(0,0)*(2*c_a2.cwiseProduct(c_posDotM)
                                                        + c_a.cwiseProduct(3*c_a.cwiseProduct(c_posDotM) - c_mxR2)
                                                        - c_r2.cwiseProduct(2*c_amx+c_bmy+c_cmz)
                                                        )
                                            +s.capDeltaPos(1,0)*(c_a.cwiseProduct(2*c_b.cwiseProduct(c_posDotM) - c_myR2)
                                                         + c_b.cwiseProduct(3*c_a.cwiseProduct(c_posDotM) - c_mxR2)
                                                         )
                                            +s.capDeltaPos(2,0)*(c_a.cwiseProduct(2*c_c.cwiseProduct(c_posDotM) - c_mzR2)
                                                         + c_c.cwiseProduct(3*c_a.cwiseProduct(c_posDotM) - c_mxR2)
                                                         )
                                             )
//                                        +2*c_r2.cwiseProduct(s.capDeltaOrient(0, 0)*(3*c_a.cwiseProduct(_aq2bq1cq0)+c_q2*c_r2)
//                                                             +s.capDeltaOrient(1, 0)*(3*c_a.cwiseProduct(aq3bq0_cq1)-c_q3*c_r2)
//                                                             -s.capDeltaOrient(2, 0)*(3*c_a.cwiseProduct(aq0_bq3cq2)-c_q0*c_r2)
//                                                             +s.capDeltaOrient(3, 0)*(3*c_a.cwiseProduct(aq1bq2cq3)-c_q1*c_r2))
                                        ).cwiseQuotient(c_r7);

    deltaFlux.col(1) = -3*b_moment*scale*(dpos_bigMag(0)*(b_a.cwiseProduct(3*b_b.cwiseProduct(b_posDotM) - b_myR2)
                                            + b_b.cwiseProduct(2*b_a.cwiseProduct(b_posDotM) - b_mxR2)
                                            )
                                        +dpos_bigMag(1)*(2*b_b2.cwiseProduct(b_posDotM)
                                             + b_b.cwiseProduct(3*b_b.cwiseProduct(b_posDotM) - b_myR2)
                                             - b_r2.cwiseProduct(b_amx+2*b_bmy+b_cmz)
                                            )
                                        +dpos_bigMag(2)*(b_b.cwiseProduct(2*b_c.cwiseProduct(b_posDotM) - b_mzR2)
                                             +b_c.cwiseProduct(3*b_b.cwiseProduct(b_posDotM) - b_myR2)
                                            )
                                        ).cwiseQuotient(b_r7)
                       + c_moment*scale*(-3*(s.capDeltaPos(0,0)*(c_a.cwiseProduct(3*c_b.cwiseProduct(c_posDotM) - c_myR2)
                                                        + c_b.cwiseProduct(2*c_a.cwiseProduct(c_posDotM) - c_mxR2)
                                                        )
                                            +s.capDeltaPos(1,0)*(2*c_b2.cwiseProduct(c_posDotM)
                                                         + c_b.cwiseProduct(3*c_b.cwiseProduct(c_posDotM) - c_myR2)
                                                         - c_r2.cwiseProduct(c_amx+2*c_bmy+c_cmz)
                                                        )
                                            +s.capDeltaPos(2,0)*(c_b.cwiseProduct(2*c_c.cwiseProduct(c_posDotM) - c_mzR2)
                                                         +c_c.cwiseProduct(3*c_b.cwiseProduct(c_posDotM) - c_myR2)
                                                        )
                                              )
//                                         +2*c_r2.cwiseProduct(s.capDeltaOrient(0, 0)*(3*c_b.cwiseProduct(_aq2bq1cq0)-c_q1*c_r2)
//                                                            +s.capDeltaOrient(1, 0)*(3*c_b.cwiseProduct(aq3bq0_cq1)-c_q0*c_r2)
//                                                            -s.capDeltaOrient(2, 0)*(3*c_b.cwiseProduct(aq0_bq3cq2)+c_q3*c_r2)
//                                                            +s.capDeltaOrient(3, 0)*(3*c_b.cwiseProduct(aq1bq2cq3)-c_q2*c_r2))
                                          ).cwiseQuotient(c_r7);

    deltaFlux.col(2) = -3*b_moment*scale*(dpos_bigMag(0)*(b_a.cwiseProduct(3*b_c.cwiseProduct(b_posDotM) - b_mzR2)
                                            + b_c.cwiseProduct(2*b_a.cwiseProduct(b_posDotM) - b_mxR2)
                                            )
                                        +dpos_bigMag(1)*(b_b.cwiseProduct(3*b_c.cwiseProduct(b_posDotM) - b_mzR2)
                                             + b_c.cwiseProduct(2*b_b.cwiseProduct(b_posDotM) - b_myR2)
                                            )
                                        +dpos_bigMag(2)*(2*b_c2.cwiseProduct(b_posDotM)
                                             + b_c.cwiseProduct(3*b_c.cwiseProduct(b_posDotM) - b_mzR2)
                                             - b_r2.cwiseProduct(b_amx+b_bmy+2*b_cmz)
                                             )
                                       ).cwiseQuotient(b_r7)
                       +c_moment*scale*(-3*(s.capDeltaPos(0,0)*(c_a.cwiseProduct(3*c_c.cwiseProduct(c_posDotM) - c_mzR2)
                                                        + c_c.cwiseProduct(2*c_a.cwiseProduct(c_posDotM) - c_mxR2)
                                                        )
                                           +s.capDeltaPos(1,0)*(c_b.cwiseProduct(3*c_c.cwiseProduct(c_posDotM) - c_mzR2)
                                                         + c_c.cwiseProduct(2*c_b.cwiseProduct(c_posDotM) - c_myR2)
                                                        )
                                           +s.capDeltaPos(2,0)*(2*c_c2.cwiseProduct(c_posDotM)
                                                         + c_c.cwiseProduct(3*b_c.cwiseProduct(c_posDotM) - c_mzR2)
                                                         - c_r2.cwiseProduct(c_amx+c_bmy+2*c_cmz)
                                                         )
                                            )
//                                        +2*c_r2.cwiseProduct(s.capDeltaOrient(0, 0)*(3*c_c.cwiseProduct(_aq2bq1cq0)-c_q0*c_r2)
//                                                           +s.capDeltaOrient(1, 0)*(3*c_c.cwiseProduct(aq3bq0_cq1)+c_q1*c_r2)
//                                                           -s.capDeltaOrient(2, 0)*(3*c_c.cwiseProduct(aq0_bq3cq2)-c_q2*c_r2)
//                                                           +s.capDeltaOrient(3, 0)*(3*c_c.cwiseProduct(aq1bq2cq3)-c_q3*c_r2))
                                       ).cwiseQuotient(c_r7);

    Map<MatrixXd> dFluxReshape(deltaFlux.data(), numOfTotalData, 1);

    deltaEstimateFlux = dFluxReshape;
    return dFluxReshape; //返回磁场变化值
}

ukfom::ukf<magState>::cov MagLocalizerPro::ukf_Q()
{
    ukfom::ukf<magState>::cov cov = ukfom::ukf<magState>::cov::Zero();
    cov.diagonal().setConstant(0.1);
    return cov;
}

MatrixXd MagLocalizerPro::ukf_R()
{
    return 500*MatrixXd::Identity(dimMeasurement, dimMeasurement);
}

MagLocalizerPro::MagLocalizerPro(QObject *parent, MagSensorParams *params) : QObject(parent)
{
    isReadyToCalibrate = false;
    int dim_z = MagSensorParams::NumOfFluxDataInTotal;
    gradienter = new GradientSolver(1, dt);
    const ukfom::ukf<magState>::cov init_cov
            = 10 * ukfom::ukf<magState>::cov::Identity();

    initState.bigMagMoment << bigMagMoment;
    initState.bigMagPos << 0, 0, 50;
    initState.capDeltaPos << 0.1, 0.001, 0.001;
    //initState.capDeltaOrient << 0.001, 0.001, 0.001, 0.001;
    initState.capPos << 0, 0, 5;


    kf = new ukfom::ukf<magState>(initState, init_cov);
    deltaEstimateFlux = MatrixXd::Zero(dim_z, 1);
    rawFlux = deltaEstimateFlux;
    deltaRawFlux = deltaEstimateFlux;
    sensorLoc = params->getGeometryMatrix();
}

void MagLocalizerPro::updateBigMagState(float x, float y, float z, matXdArgConst &rawData)
{
    Vector3d posNew(x, y, z);
    dpos_bigMag = posNew - pos_bigMag;
    pos_bigMag = posNew;


    MatrixXd filterout = gradienter->filter(rawData);
    deltaRawFlux = filterout - rawFlux;
    rawFlux = filterout;
    if(!isReadyToCalibrate)
    {
        isReadyToCalibrate = true;
    }
    else
    {
        start(deltaRawFlux);
        cout<<kf->mu()<<endl;
    }
}

void MagLocalizerPro::start(const BuildMeasurement& measurement)
{
    kf->predict(ukf_fx, ukf_Q);
    kf->update(measurement, ukf_hx, ukf_R);

}
