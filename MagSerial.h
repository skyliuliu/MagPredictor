#ifndef MAGSERIAL_H
#define MAGSERIAL_H

#include <QObject>
#include <string>
#include <iostream>
#include "QThread"
#include "QtSerialPort/QSerialPort"
#include <Eigen/Dense>
#include <QTimer>
#include "filtercommon.h"
#include "magsensorparams.hpp"


using namespace std;
using namespace Eigen;

class MagSerial : public QObject
{
    Q_OBJECT
public:
    explicit MagSerial(QObject *parent = nullptr, MagSensorParams *params=new MagSensorParams());
    ~MagSerial();

private:
    MagSensorParams *serialParam;
    QThread *serialThread;
    QSerialPort *serial;
    QTimer *comWatchTimer;

private:
    //mag sensor data definition
    QVector<double> currents;
    bool isReadyToSendData;
    inline bool isDataSizeValid(const QByteArray& data)const;
    inline bool isDataIDValid(const int& ID)const;
    void unpackData(const QByteArray& dataPack);

signals:
    void isActive(bool);
    void currentCOMChanged(QStringList);
    void serialDataReadySend(QVector<double>);

public slots:
    void readCurr();
    void initCOM(QString);
    void initCOM_Watcher();
    void closeCOM(QString);
    void wirteSendSer(QString);
};

#endif // MAGSERIAL_H
