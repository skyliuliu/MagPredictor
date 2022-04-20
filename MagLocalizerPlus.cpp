#include "MagLocalizerPlus.h"
#include <QElapsedTimer>
//#include <extensionsystem/pluginmanager.h>
//#include "uavobjectmanager.h"
using namespace std;

double capEstimator::capsuleMoment = 0.29;
double capEstimator::dt = 0.03;
double capEstimator::Q = 0.01;
double capEstimator::R = 500;
double capEstimator::scale = 1e6;
Eigen::MatrixXd capEstimator::SensorLoc;
capEstimator::BuildMeasurement capEstimator::predictedData;

MagLocalizerPlus::MagLocalizerPlus(QObject *parent,
                                   MagSensorParams *params)
    : QObject(parent)
{
    caliStatus = 0;
    maxCaliTimes = 300;
    status = calibrating;
    serialDataReady = false;
    bigMagDataReady = false;
    predictor = new capEstimator(params);
    predictor->setTimeInterval(0.03);

    localizerThread = new QThread();
    moveToThread(localizerThread);
    connect(localizerThread, SIGNAL(started()),this,SLOT(init()));
    localizerThread->start();
}

MagLocalizerPlus::~MagLocalizerPlus()
{
    localizerTimer->stop();
    localizerThread->quit();
    localizerThread->wait();
}

void MagLocalizerPlus::serialIn(MatrixXd rawMagData)
{
    this->rawMagData = rawMagData;
    if(!serialDataReady)
    {
        serialDataReady = true;
    }
    if(status==stopped)
    {
        status=readyToStart;
    }
}

void MagLocalizerPlus::init()
{
    initSmoother();
    caliTimer = new QTimer();
    connect(caliTimer, SIGNAL(timeout()),this,SLOT(calibrateData()));
    localizerTimer = new QTimer();
    connect(localizerTimer, SIGNAL(timeout()), this, SLOT(updatePrediction()));

    /*
    //将大磁球信息拉下来
    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    UAVObjectManager *objManager = pm->getObject<UAVObjectManager>();
    Q_ASSERT(objManager);
    Si_MotionCtrol *bigMag = dynamic_cast<Si_MotionCtrol *>(objManager->getObject(QString("Si_MotionCtrol")));
    if(bigMag != NULL)
    {
        connect(bigMag, SIGNAL(objectUpdated(UAVObject *)),
                this, SLOT(updateBigMagState(UAVObject *)));
    }
    */
}

void MagLocalizerPlus::initSmoother()
{
    int dim_kfState = capEstimator::dimMeasurement*2;
    int dim_RawMagData = capEstimator::dimMeasurement;
    smoother = new KalmanFilter(dim_kfState, dim_RawMagData);
    for(int i=0;i<dim_kfState;i+=2){
        smoother->x(i, 0) = 1.0;
    }
    smoother->H = MatrixXd::Zero(dim_RawMagData, dim_kfState);
    for(int i=0;i<dim_RawMagData;i++){
        smoother->F(2*i, 1+2*i) = 0.03;
        smoother->H(i, 2*i) = 1;
    }
    smoother->P *= 200;
    smoother->R = 3*MatrixXd::Identity(dim_RawMagData, dim_RawMagData);
    smoother->Q = 0.05*MatrixXd::Identity(dim_kfState, dim_kfState);
}

void MagLocalizerPlus::calibrateData()
{
    if(caliStatus<maxCaliTimes){
        smoother->predict();
        smoother->update(rawMagData);
        Map<MatrixXd> smootherXreshape(smoother->x.data(), 2, capEstimator::dimMeasurement);
        filteredData = smootherXreshape.row(0);
        caliMagData += filteredData;
        caliStatus += 1;
        emit sendCaliStatus(caliStatus);
        if(status != calibrating)
        {
            status = calibrating;
        }
    }
    else if(caliStatus==maxCaliTimes){
        caliMagData /= (caliStatus);
        caliStatus += 1;
        emit sendCaliStatus(caliStatus);
        status = readyToStart;
        caliTimer->stop();
    }
}

void MagLocalizerPlus::reCalibrate()
{
    predictor->clearAll();
    localizerTimer->stop(); // stop predict timer
    caliTimer->start(30);//30ms
    caliMagData.fill(0);
    caliStatus = 0; // reset counter
}

void MagLocalizerPlus::updatePrediction()
{
    if(serialDataReady==true)
    {
        serialDataReady = false;
        predictor->estimate(rawMagData-caliMagData);
        //cout<<predictor->getPosAndRotVector().transpose()<<endl;
        emit sendMagState(predictor->getPosAndRotVector());
        emit sendFluxData(rawMagData, filteredData,
                          rawMagData-caliMagData, predictor->getPredictedData());
    }
}

/*
void MagLocalizerPlus::updateBigMagState(UAVObject *obj)
{
    Si_MotionCtrol *p_bigMag = dynamic_cast<Si_MotionCtrol *>(obj);
    if(p_bigMag != NULL){
        const quint8 *status = p_bigMag->getData().Move;
        const float *angle = p_bigMag->getData().Angle;
        const float *loc = p_bigMag->getData().Position;
        //当磁球角度无变化，开启定位
        if(serialDataReady)
        {
            BigMagStatus.updateStatus(loc, angle);
            if(BigMagStatus.isXYZmove)
            {
            }
        }
    }
}
*/

/**
 * @brief MagLocalizerPlus::continuousStart
 * 槽函数链接contineous 按钮
 */
void MagLocalizerPlus::continuousStart()
{
    if(status==readyToStart){
        localizerTimer->start(1000*predictor->getTimeInterval());//30ms
        status = started;
    }
}

void MagLocalizerPlus::stop()
{
    if(localizerTimer->isActive())
    {
        localizerTimer->stop();
        caliTimer->stop();
        status=stopped;
    }
}
