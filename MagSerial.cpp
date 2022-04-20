#include "MagSerial.h"
#include <stdio.h>
#include <QDebug>
#include <QSerialPortInfo>

/*
 *  求补码
 */
inline int get2sComplement(const int& hex)
{
    if((hex & 0x8000) == 0x8000){
        return -(int)(0xffff-hex+1);
    }
    else{
        return (int)hex;
    }
}

MagSerial::MagSerial(QObject *parent, MagSensorParams *params)
    :QObject(parent)
{
    serial = new QSerialPort();
    serialThread = new QThread();
    serialParam = params;

    isReadyToSendData = false;

    //specify the parameters
    MagRealData = MatrixXd::Zero(params->NumOfFluxDataInTotal,1);

    this->moveToThread(serialThread);
    serial->moveToThread(serialThread);
    connect(serialThread,SIGNAL(finished()),serial,SLOT(deleteLater()));
    connect(serialThread,SIGNAL(started()),this,SLOT(initCOM_Watcher()));
    serialThread->start();
}

MagSerial::~MagSerial(){
    closeCOM();
    comWatchTimer->stop();
    serialThread->quit();
    serialThread->wait();
    qDebug()<<__FUNCTION__<<"stop";
}

void MagSerial::getPortList()
{
    QStringList tempPort;
    foreach(const QSerialPortInfo &info,QSerialPortInfo::availablePorts())
    {
        tempPort<<info.portName();
    }

    //检查是否有删除掉的port
    if(serialPortName.size()>tempPort.size()){
        foreach(const QString &port, serialPortName){
            if(!tempPort.contains(port)){
                serialPortName.removeOne(port);
                emit currentCOMChanged(serialPortName);
            }
        }
    }
    //检查是否有需要更新新增的port
    else{
        foreach(const QString &port, tempPort){
            if(!serialPortName.contains(port)){
                serialPortName<<port;
                emit currentCOMChanged(serialPortName);
            }
        }
    }

}

void MagSerial::initCOM_Watcher()
{
    comWatchTimer = new QTimer();
    connect(comWatchTimer,SIGNAL(timeout()),this,SLOT(getPortList()));
    comWatchTimer->start(1000);
}

void MagSerial::initCOM(QString comName)
{
    if(serial->isOpen()) closeCOM();
    comWatchTimer->stop();
    serial->setPortName(comName);
    if(!serial->open(QIODevice::ReadWrite))//用ReadWrite 的模式尝试打开串口
    {
        qDebug()<<"Failed to open COM!";
        emit isActive(false);
        return;
    }
    emit isActive(true);
    qDebug()<<"Open COM OK!";
    serial->setBaudRate(QSerialPort::Baud115200,QSerialPort::AllDirections);//波特率&读写方向
    serial->setDataBits(QSerialPort::Data8);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    qRegisterMetaType<QSerialPort::SerialPortError>("QSerialPort::SerialPortError");
    connect(serial,SIGNAL(readyRead()),this,SLOT(doRead()));
}

void MagSerial::handleError(QSerialPort::SerialPortError error)
{
    if(error == QSerialPort::ResourceError){
        //qDebug()<<"串口断开连接,关闭串口线程";
        closeCOM();
    }
}

void MagSerial::closeCOM()
{
    disconnect(serial, SIGNAL(readyRead()), 0, 0); //断开所有和读取相关的信号
    if(!comWatchTimer->isActive()){
        comWatchTimer->start(1000); //打开监视器
    }
    serial->clear();
    serial->close();
    if(serial->isOpen()==false)
    {
        emit isActive(false);
    }
}

void MagSerial::doRead()
{
    if(serial->isReadable())
    {
        QByteArray info = serial->readAll();
        if(!isDataSizeValid(info))return;
        const char *data = info.constData(); //无需手动释放内存，指向常量
        for(int i=0;i<info.size();i+=serialParam->NumOfBytesPerSensor)
        {
            int ID = (int)(unsigned char)data[i+6];
            int BxID = ID - 1;
            int ByID = ID - 1 + serialParam->NumOfSensors;
            int BzID = ID - 1 + 2*serialParam->NumOfSensors;
            if(BzID < serialParam->NumOfFluxDataInTotal)
            {
                MagRealData(BxID,0) = -1.5*get2sComplement(((int)(unsigned char)data[i+1]<<8)^((int)(unsigned char)data[i]));
                MagRealData(ByID,0) = 1.5*get2sComplement(((int)(unsigned char)data[i+3]<<8)^((int)(unsigned char)data[i+2]));
                MagRealData(BzID,0) = 1.5*get2sComplement(((int)(unsigned char)data[i+5]<<8)^((int)(unsigned char)data[i+4]));
                if(!isReadyToSendData)isReadyToSendData=true;
            }
            else
            {
                qDebug()<<__FUNCTION__<<"-Wrong ID read from PCBs"<<" ID:"<<ID;
                if(isReadyToSendData)isReadyToSendData=false;
            }
        }
        if(isReadyToSendData)
            emit serialDataReadySend(MagRealData);
    }
}

inline bool MagSerial::isDataSizeValid(const QByteArray &data)const
{
    return data.size()%serialParam->NumOfBytesPerSensor == 0;
}

inline bool MagSerial::isDataIDValid(const int &ID)const
{
    return serialParam->SensorIDLIB.contains(ID);
}

void MagSerial::unpackData(const QByteArray &dataPack)
{
    if(!isDataSizeValid(dataPack))
    {
        qDebug()<<__FUNCTION__<<" -";
        return;
    }
    const char *data = dataPack.constData(); //无需手动释放内存，指向常量

}
