#ifndef MAGSENSORPARAMS_H
#define MAGSENSORPARAMS_H
#include <Eigen/Dense>
#include <QObject>
#include <QVector>
using namespace Eigen;
#include <QDebug>

#ifndef NUM_OF_SENSORSET
#define NUM_OF_SENSORSET 1
#endif

#ifndef MAG_PI
#define MAG_PI 3.1415926536
#endif

struct SensorSet{
private:
    MatrixXd sensorLoc;

public:
    float cx;
    float cy;
    float cz;
    float interval;

    void translate(float x, float y, float z)
    {
        cx+=x;
        cy+=y;
        cz+=z;
        MatrixXd transMat = MatrixXd::Zero(9,3);
        transMat.col(0).fill(x);
        transMat.col(1).fill(y);
        transMat.col(2).fill(z);
        sensorLoc += transMat;
    }

    /**
     * @brief rotate 旋转板子组
     * @param angleDegree 旋转角度
     * @param rotDistance 旋转轴距离中心点的距离，一般是在x轴上
     */
    void rotate(double angleDegree, double rotDistance)
    {
        MatrixXd rotDis = MatrixXd::Zero(9, 1);
        rotDis.fill(rotDistance);
        sensorLoc.col(0) = (sensorLoc.col(0)-rotDis)*cos(angleDegree*MAG_PI/180)+rotDis;
        sensorLoc.col(2) += (sensorLoc.col(0)-rotDis)*sin(angleDegree*MAG_PI/180);
    }

    MatrixXd geometry()const
    {
        return sensorLoc;
    }

    void resetLocation(float c_x, float c_y, float c_z, float Interval)
    {
        cx=c_x;cy=c_y;cz=c_z;interval=Interval;
        sensorLoc<<cx-interval,cy+interval,cz, cx, cy+interval,cz, cx+interval, cy+interval, cz,
                   cx-interval,cy,cz,          cx, cy, cz,         cx+interval, cy, cz,
                   cx-interval,cy-interval,cz, cx, cy-interval,cz, cx+interval, cy-interval, cz;
    }

    SensorSet(){
        cx=0;cy=0;cz=0;interval=6;
        sensorLoc = MatrixXd::Zero(9, 3);
        resetLocation(0, 0, 0, 6);
        }
    SensorSet(float c_x, float c_y, float c_z, float Interval)
        :cx(c_x),cy(c_y),cz(c_z),interval(Interval){
        sensorLoc = MatrixXd::Zero(9, 3);
        resetLocation(c_x, c_y, c_z, Interval);
        }
};

template<int SensorSetNum>
struct MagSensorParameters
{
public:
    enum
    {
        /*
         *定义编译期间的int枚举常量，
         * VS2012不支持constexpr
        */
        NumOfSensorSets = SensorSetNum, //阵列数
        NumOfSensors = SensorSetNum*9,  //sensor数
        NumOfSensorsPerSet = 9,         //每个阵列的sensor数
        NumOfBytesPerSensor = 7,        //每个sensor输出数据的字节数
        NumOfBytesPerSensorSet = 63,    //每个阵列输出数据的字节数
        NumOfAxisPerSensor = 3,         //sensor的轴数
        NumOfFluxDataInTotal=NumOfSensors*NumOfAxisPerSensor  //所有阵列测量的总轴数
    };
    MagSensorParameters(){
        SensorSets = new SensorSet[NumOfSensorSets];
        //初始化sensor set ID
        for(int i=0;i<NumOfSensorSets;i++)
        {
            SensorSetID.push_back(i);
        }
        //初始化sensors ID
        for(int i=0;i<NumOfSensors;i++)
        {
            SensorIDLIB.push_back(i+1);
        }
    }
    SensorSet *SensorSets;
    QVector<int> SensorIDLIB;
    QVector<int> SensorSetID;

    /**
     * @brief setSensorSetPos
     * 单位：cm
     * @param sensorSetID
     * @param cx
     * @param cy
     * @param cz
     * @param interval
     */
    void setSensorSetPos(int sensorSetID,
                         float cx, float cy, float cz,
                         float interval)
    {
        if(SensorSetID.contains(sensorSetID))
        {
            SensorSets[sensorSetID].resetLocation(cx,cy,cz,interval);
        }
        else
        {
            qDebug()<<__FUNCTION__
                    <<"sensor set ID should not be larger than "
                   <<SensorSetID.size();
            return;
        }
    }
    void translateSensorSet(int sensorSetID, float x, float y, float z)
    {
        if(SensorSetID.contains(sensorSetID))
        {
            SensorSets[sensorSetID].translate(x, y, z);
        }
        else
        {
            qDebug()<<__FUNCTION__
                    <<"sensor set ID should not be larger than "<<SensorSetID.size();
            return;
        }
    }

    void rotateSensorSet(int sensorSetID,
                         double angleDegree, double rotDistance)
    {
        if(SensorSetID.contains(sensorSetID))
        {
            SensorSets[sensorSetID].rotate(angleDegree, rotDistance);
        }
        else
        {
            qDebug()<<__FUNCTION__
                    <<"sensor set ID should not be larger than "<<SensorSetID.size();
            return;
        }
    }
    MatrixXd getGeometryMatrix()const
    {
        MatrixXd geometry = MatrixXd::Zero(NumOfSensors, 3);
        for(int i=0;i<NumOfSensorSets;i++){
            geometry.middleRows(i*NumOfSensorsPerSet,NumOfSensorsPerSet)
                    =SensorSets[i].geometry();
        }
        return geometry;
    }
};
typedef MagSensorParameters<NUM_OF_SENSORSET> MagSensorParams;

#endif // MAGSENSORPARAMS_H
