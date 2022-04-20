/**
 ******************************************************************************
 *
 * @file       MagPredictorWidget.cpp
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup ModelViewPlugin ModelView Plugin
 * @{
 * @brief A gadget that displays a 3D representation of the UAV
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include "QtDebug"
#ifdef __APPLE__
    #include "OpenGL/OpenGL.h"
#endif
#include "MagPredictorwidget.h"
#include "ui_MagPredictorwidget.h"

#include <QWheelEvent>
//#include <coreplugin/icore.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <ctime>
#include <QDebug>
#include "MagLocalizerPlus.h"

using namespace Eigen;

MagPredictorWidget::MagPredictorWidget(QWidget *parent) : QWidget(parent),
    ui(new Ui::MagPredictorWidget)
{
    ui->setupUi(this);
    //this->showFullScreen();

    this->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)),
            this, SLOT(onPopUpMenu(const QPoint &)));

#if 1
    Eigen::initParallel();
    //qDebug()<<__FUNCTION__<<QThread::currentThreadId();
    myParam = new MagSensorParams();
    myParam->setSensorSetPos(0,0,0,0, 12);
//    myParam->setSensorSetPos(1,40.5,0,0, 12);
//    myParam->setSensorSetPos(2,-40.5,0,0, 12);


    //serial port init
    mySer = new MagSerial(0,myParam);
    //localizer init
    myLoc= new MagLocalizerPlus(0, myParam);
    //slot connect
    //connect(ui->COMList,SIGNAL(showList()),mySer,SLOT(getPortList()));
    connect(ui->COMBtn,SIGNAL(clicked()),this,SLOT(ctrlCOM()));
    connect(mySer,SIGNAL(isActive(bool)),this,SLOT(handleCOMOpen(bool)));
    connect(this,SIGNAL(currentCOM(QString)),mySer,SLOT(initCOM(QString)));
    connect(mySer,SIGNAL(currentCOMChanged(QStringList)),
            ui->COMList,SLOT(updateListItems(QStringList)));
    connect(this, SIGNAL(stopCOM()), mySer, SLOT(closeCOM()));
    connect(this, SIGNAL(stopCOM()), myLoc, SLOT(stop()));

    qRegisterMetaType<MatrixXd>("MatrixXd");
    qRegisterMetaType<std::vector<MatrixXd>>("std::vector<MatrixXd>");
    connect(mySer, SIGNAL(serialDataReadySend(MatrixXd)),this->myLoc, SLOT(serialIn(MatrixXd)));

    connect(myLoc,SIGNAL(sendFluxData(MatrixXd,MatrixXd,MatrixXd,MatrixXd)),
            this,SLOT(getPlotData(MatrixXd,MatrixXd,MatrixXd,MatrixXd)));
    connect(myLoc,SIGNAL(sendMagState(MatrixXd)),this,SLOT(getLocData(MatrixXd)));
    connect(myLoc,SIGNAL(sendCaliStatus(int)),ui->caliProgressBar,SLOT(setValue(int)));

    //slot connect for cmd
    connect(ui->recalibrateBtn,SIGNAL(clicked(bool)),myLoc,SLOT(reCalibrate()));
    connect(ui->ctnStartBtn,SIGNAL(clicked(bool)),myLoc,SLOT(continuousStart()));
    //Mode configuration
    //slave select
    //COM configuration
    COM_state = disconnected;
    ui->COMList->addItem("None");
    currentSlave = 0;
    for(int i=0;i<myParam->NumOfSensors;i++){
        ui->SlaveList->addItem("Slave "+QString::number(i+1));
    }

    connect(ui->SlaveList, SIGNAL(currentIndexChanged(int)), this, SLOT(selectSlave(int)));

    currentMode = Braw_vs_Bkf;
    ui->ModeList->addItem("Filtering");
    ui->ModeList->addItem("Prediction");
    connect(ui->ModeList, SIGNAL(currentIndexChanged(int)),this,SLOT(selectMode(int)));

    //Plot
    initCustomPlot(ui->ADCplot, 0);
    init2CustomPlot(ui->VmsPlot, true);
    initCustomPlot(ui->GyroPlot, 1);
    initCustomPlot(ui->AccPlot, 2);
    init2CustomPlot(ui->OutputPlot, false);

    // 显示初始位姿
    Matrix<double, 7, 1> state0;
    state0 << 0, 0, 2, 1, 1, 1, 0;
    getLocData(state0);
#endif
}

MagPredictorWidget::~MagPredictorWidget()
{
    delete ui;
}

GLWidget *MagPredictorWidget::glwidget()
{
    return ui->glwidget_Mag3D;
}

void MagPredictorWidget::initCustomPlot(QCustomPlot *Plot, int sensorId)
{
    Plot->setInteractions(QCP::iRangeDrag //可平移
                          | QCP::iRangeZoom //可滚轮缩放
                          | QCP::iSelectLegend);//可选中图例
    Plot->addGraph();
#if 1
    if(sensorId){
        Plot->graph(0)->setLineStyle(QCPGraph::lsLine);
        QPen pen(Qt::red);
        Plot->graph(0)->setPen(pen);
        Plot->graph(0)->setName(QStringLiteral("X"));

        Plot->addGraph();
        Plot->graph(1)->setLineStyle(QCPGraph::lsLine);
        QPen pen2(Qt::green);
        Plot->graph(1)->setPen(pen2);
        Plot->graph(1)->setName(QStringLiteral("Y"));

        Plot->addGraph();
        Plot->graph(2)->setLineStyle(QCPGraph::lsLine);
        QPen pen3(Qt::white);
        Plot->graph(2)->setPen(pen3);
        Plot->graph(2)->setName(QStringLiteral("Z"));

        auto yLabel = sensorId == 2 ? QStringLiteral("加速度 (mg)"): QStringLiteral("角速度 (deg/s)");
        Plot->yAxis->setLabel(yLabel);
        Plot->yAxis->setLabelColor(Qt::white);
    }else{
        QPen pen(Qt::white);
        Plot->graph(0)->setPen(pen);
        Plot->yAxis->setLabel("V (uV)");
        Plot->yAxis->setLabelColor(Qt::white);
    }

    //坐标轴使用时间刻度
    QSharedPointer<QCPAxisTickerDateTime> dateTicker(new QCPAxisTickerDateTime);
    dateTicker->setDateTimeFormat("hh:mm:ss");
    Plot->xAxis->setTicker(dateTicker);

    //key的单位是秒
    double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
    //Plot->graph(0)->addData(key, 0);
    Plot->xAxis->setRange(key, 5, Qt::AlignRight);

    Plot->legend->setVisible(true);
    Plot->legend->setBrush(QBrush(QColor(80, 80, 80, 150)));
    Plot->legend->setTextColor(Qt::white);
#endif

    // make left and bottom axes transfer their ranges to right and top axes:
    qRegisterMetaType<QCPRange>("QCPRange");
    connect(Plot->xAxis, SIGNAL(rangeChanged(QCPRange)),Plot->xAxis2, SLOT(setRange(QCPRange)));
    connect(Plot->yAxis, SIGNAL(rangeChanged(QCPRange)),Plot->yAxis2, SLOT(setRange(QCPRange)));

    //Plot->addLayer("abovemain", Plot->layer("main"), QCustomPlot::limAbove);
    Plot->addLayer("belowmain", Plot->layer("main"), QCustomPlot::limBelow);
    Plot->xAxis->grid()->setLayer("belowmain");
    Plot->yAxis->grid()->setLayer("belowmain");
    // set some pens, brushes and backgrounds:
    Plot->xAxis->setBasePen(QPen(Qt::white, 1));
    Plot->yAxis->setBasePen(QPen(Qt::white, 1));
    Plot->xAxis->setTickPen(QPen(Qt::white, 1));
    Plot->yAxis->setTickPen(QPen(Qt::white, 1));
    Plot->xAxis->setSubTickPen(QPen(Qt::white, 1));
    Plot->yAxis->setSubTickPen(QPen(Qt::white, 1));
    Plot->xAxis->setTickLabelColor(Qt::white);
    Plot->yAxis->setTickLabelColor(Qt::white);
    Plot->xAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
    Plot->yAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
    Plot->xAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
    Plot->yAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
    Plot->xAxis->grid()->setSubGridVisible(true);
    Plot->yAxis->grid()->setSubGridVisible(true);
    Plot->xAxis->grid()->setZeroLinePen(Qt::NoPen);
    Plot->yAxis->grid()->setZeroLinePen(Qt::NoPen);
    Plot->xAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);
    Plot->yAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);
    QLinearGradient plotGradient;
    plotGradient.setStart(0, 0);
    plotGradient.setFinalStop(0, 350);
    plotGradient.setColorAt(0, QColor(80, 80, 80));
    plotGradient.setColorAt(1, QColor(50, 50, 50));
    Plot->setBackground(plotGradient);
    QLinearGradient axisRectGradient;
    axisRectGradient.setStart(0, 0);
    axisRectGradient.setFinalStop(0, 350);
    axisRectGradient.setColorAt(0, QColor(80, 80, 80));
    axisRectGradient.setColorAt(1, QColor(30, 30, 30));
    Plot->axisRect()->setBackground(axisRectGradient);
}

void MagPredictorWidget::init2CustomPlot(QCustomPlot *plot, bool isVms)
{
    plot->setInteractions(QCP::iRangeDrag //可平移
                          | QCP::iRangeZoom //可滚轮缩放
                          | QCP::iSelectLegend);//可选中图例

    QCPBars *currentBar = new QCPBars(plot->xAxis, plot->yAxis);
    currentBar->setName("currents");
//    QPen pen = QPen(Qt::green);
//    currentBar->setPen(pen);
//    currentBar->setBrush(pen);

    // x axis
    QVector<double> ticks;
    QVector<QString> labels;
    for (int i=1; i<17; ++i) {
        ticks.push_back(i);
        labels.push_back(QString::number(i));
    }
    QSharedPointer<QCPAxisTickerText> textTicker(new QCPAxisTickerText);
    textTicker->addTicks(ticks, labels);
    plot->xAxis->setTicker(textTicker);
    plot->xAxis->setRange(0, 17);
    plot->xAxis->setLabel(QStringLiteral("线圈编号"));

    // y axis
    auto yLabel = isVms ? QStringLiteral("inducedVoltage (uV)") : QStringLiteral("coil current (A)");
    plot->yAxis->setLabel(yLabel);

    QVector<double> currents;
    currents << 2.21 << 2.22 << 2.31<< 2.39<< 2.33<< 2.31<< 2.29<< 2.34<< 2.29<< 2.38<< 2.36<< 2.31<< 2.35<< 2.41<< 2.42<< 2.35;
    currentBar->setData(ticks, currents);

    for (int i=0; i<16; ++i) {
        QCPItemText *textLabel = new QCPItemText(plot);
        textLabel->setPositionAlignment(Qt::AlignTop|Qt::AlignHCenter);
        textLabel->position->setType(QCPItemPosition::ptPlotCoords);
        textLabel->position->setCoords(i + 1, currents[i] + 0.2);
        textLabel->setText(QString::number(currents[i]));
        textLabel->setPen(QPen(Qt::blue));
    }

    //plot->legend->setVisible(true);
}

void MagPredictorWidget::plotData(QCustomPlot *Plot, double in1, double in2)
{
    static QTime time(QTime::currentTime());
    double key = time.elapsed()/1000.0; // 开始到现在的时间，单位秒
    Plot->graph(0)->addData(key, in1);
    Plot->graph(1)->addData(key, in2);
    Plot->xAxis->setRange(key, 10, Qt::AlignRight);
    Plot->replot();//绘图
}

void MagPredictorWidget::clrPlot(QCustomPlot *Plot)
{
    Plot->graph(0)->data().data()->clear();
    Plot->graph(1)->data().data()->clear();
    Plot->replot();//绘图
}

void MagPredictorWidget::onPopUpMenu(const QPoint &mousePosition)
{
    Q_UNUSED(mousePosition);

    QMenu menu;
    QAction *action = menu.addAction(tr("Options..."));
    connect(action, &QAction::triggered, this, &MagPredictorWidget::slot_showOptionDialog);
    menu.exec(QCursor::pos());
}

void MagPredictorWidget::slot_showOptionDialog()
{
    qDebug() << __FUNCTION__ << "------getObjName";
    //Core::ICore::instance()->showOptionsDialog(tr("MagPredictor"), objectName());
}

void MagPredictorWidget::getPlotData(MatrixXd rawData, MatrixXd filteredData,
                                     MatrixXd caliMagData, MatrixXd predictedData){
    double BxIn0,ByIn0,BzIn0;
    double BxIn1,ByIn1,BzIn1;

    if(currentMode==Braw_vs_Bkf){
        BxIn0 = rawData(currentSlave, 0);
        ByIn0 = rawData(currentSlave+myParam->NumOfSensors, 0);
        BzIn0 = rawData(currentSlave+2*myParam->NumOfSensors, 0);

        BxIn1 = filteredData(currentSlave, 0);
        ByIn1 = filteredData(currentSlave+myParam->NumOfSensors, 0);
        BzIn1 = filteredData(currentSlave+2*myParam->NumOfSensors, 0);
    }
    else if(currentMode==Bkf_vs_Bpre){
        BxIn0 = caliMagData(currentSlave, 0);
        ByIn0 = caliMagData(currentSlave+myParam->NumOfSensors, 0);
        BzIn0 = caliMagData(currentSlave+2*myParam->NumOfSensors, 0);

        BxIn1 = predictedData(currentSlave, 0);
        ByIn1 = predictedData(currentSlave+myParam->NumOfSensors, 0);
        BzIn1 = predictedData(currentSlave+2*myParam->NumOfSensors, 0);
    }

    //curve 0
    plotData(ui->ADCplot,BxIn0,BxIn1);
    plotData(ui->GyroPlot,ByIn0,ByIn1);
    plotData(ui->AccPlot,BzIn0,BzIn1);
}

void MagPredictorWidget::getLocData(MatrixXd x)
{
    double a = x(0, 0);
    double b = x(1, 0);
    double c = x(2, 0);
    double q0 = x(3, 0);
    double q1 = x(4, 0);
    double q2 = x(5, 0);
    double q3 = x(6, 0);

    double qNorm = q0*q0+q1*q1+q2*q2+q3*q3;

    double pitch = atan2(2*q0*q1+2*q2*q3, q0*q0-q1*q1-q2*q2+q3*q3);
    double roll = asin((2*q0*q2-2*q3*q1)/qNorm);
    double yaw = atan2(2*q0*q3+2*q1*q2, q0*q0+q1*q1-q2*q2-q3*q3);
    ui->glwidget_Mag3D->showCapsuleAngle(yaw, pitch, roll);

//    QString str = "[x y z]: "+QString::number(a)+" "
//                             +QString::number(b)+" "
//                             +QString::number(c)+" ";
//    ui->plainTextEdit->appendPlainText(str);

    //更新3D模型
    ui->glwidget_Mag3D->updateCapsuleState(-a, -b, c, q0, -q1, -q2, q3);
}

void MagPredictorWidget::ctrlCOM(){
    if(COM_state == disconnected){
        if(ui->COMList->currentIndex() != -1){
            emit currentCOM(ui->COMList->currentText());
        }
        else{
            QMessageBox::information(this, "No valid COM port detected",
                                     "Please Check the connection!!");
        }
    }
    else if(COM_state == connected){
        emit stopCOM();
    }

}

void MagPredictorWidget::handleCOMOpen(bool isOpen){
    if(isOpen){
        COM_state = connected;
        ui->COMBtn->setText(QStringLiteral("断开"));
        ui->COMList->setEnabled(false);
    }
    else{
        COM_state = disconnected;
        ui->COMBtn->setText(QStringLiteral("连接"));
        ui->COMList->setEnabled(true);
    }
}

void MagPredictorWidget::selectSlave(int index)
{

}

void MagPredictorWidget::selectMode(int index)
{

}
