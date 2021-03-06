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
    currents = QVector<double>(16, 2);

    this->moveToThread(serialThread);
    serial->moveToThread(serialThread);
    connect(serialThread,SIGNAL(finished()),serial,SLOT(deleteLater()));
    connect(serialThread,SIGNAL(started()),this,SLOT(initCOM_Watcher()));
    serialThread->start();
}

MagSerial::~MagSerial(){
    comWatchTimer->stop();
    serialThread->quit();
    serialThread->wait();
    qDebug()<<__FUNCTION__<<"stop";
}

void MagSerial::initCOM_Watcher()
{
    comWatchTimer = new QTimer();
    comWatchTimer->start(5000);
}

void MagSerial::initCOM(QString comName)
{
    if(serial->isOpen()) closeCOM(comName);
    comWatchTimer->stop();
    serial->setPortName(comName);
    if(!serial->open(QIODevice::ReadWrite))//用ReadWrite 的模式尝试打开串口
    {
        qDebug() << "Failed to open" << comName;
        emit isActive(false);
        return;
    }
    emit isActive(true);
    qDebug() << "Open" << comName << "OK!";
    serial->setBaudRate(921600, QSerialPort::AllDirections);//波特率&读写方向
    serial->setDataBits(QSerialPort::Data8);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    qRegisterMetaType<QSerialPort::SerialPortError>("QSerialPort::SerialPortError");
    connect(serial,SIGNAL(readyRead()),this,SLOT(readCurr()));
}


void MagSerial::closeCOM(QString comName)
{
    disconnect(serial, SIGNAL(readyRead()), 0, 0); //断开所有和读取相关的信号
    if(!comWatchTimer->isActive()){
        comWatchTimer->start(1000); //打开监视器
    }
    serial->clear();
    serial->close();
    qDebug() << "Close" << comName << "OK!";
}

// 获取发射端的电流
void MagSerial::readCurr()
{
    if(serial->isReadable())
    {
        QByteArray info = serial->readAll();
        QString infoStr(info.constData());

        QRegExp chxReg("(ch\\w:\\d{3})");
        QStringList chList;
        int pos = 0;
        while ((pos = chxReg.indexIn(infoStr, pos)) != -1) {
            chList << chxReg.cap(1);
            pos += chxReg.matchedLength();
        }

        if (!chList.isEmpty()) {
            QString chx = chList.back();
            char ch = chx[2].toLatin1();
            double curr = chx.mid(4, 3).toInt() * 0.01;
            if (ch > '0' && ch <= '9') currents[ch - '1'] = curr;
            else currents[ch - 'A' + 9] = curr;
            emit serialDataReadySend(currents);
        }
    }
}

void MagSerial::wirteSendSer(QString cmdStr) {
    QStringList cmdList = cmdStr.split(" ");
    QByteArray cmdArray;
    foreach(QString s, cmdList) cmdArray.append((char)s.toInt(0, 16));
    auto r = serial->write(cmdArray);

    if (r == -1) {
        qDebug() << "cmd error!";
        return;
    }

    (cmdList[2] == "1" || cmdList[2] == "01") ? qDebug() << "start working!" : qDebug() << "stop working!";

}
