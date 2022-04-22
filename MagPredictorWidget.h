#ifndef MAGPREDICTORWIDGET_H_
#define MAGPREDICTORWIDGET_H_

#include "glwidget.h"
#include "qcustomplot.h"

#include <QKeyEvent>
#include <QTimer>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "magsensorparams.hpp"
#include "MagSerial.h"
#include "MagLocalizerPlus.h"

using namespace std;

namespace Ui {
class MagPredictorWidget;
}

class MagPredictorWidget : public QWidget {
    Q_OBJECT

public:
    MagPredictorWidget(QWidget *parent = 0);
    ~MagPredictorWidget();

    GLWidget *glwidget();

private:
    Ui::MagPredictorWidget *ui;
    QCPBars *currentBar;
    QVector<double> currentsPlotTicks;
    QVector<double> vmsPlotTicks;
    QVector<QCPItemText*> textLabels;

    int currentSlave;

    MagSensorParams *myParam;
    //MagLocalizer *myLoc;
    MagSerial *sendSer;
    MagSerial *recvSer;
    MagLocalizerPlus *myLoc;

    void initCustomPlot(QCustomPlot *Plot, int sensorId);
    void init2CustomPlot(QCustomPlot *plot, bool isVms);
    void plotXYZPos(QCustomPlot *Plot, double x, double y, double z);
    void plotData(QCustomPlot *Plot, double in1, double in2);
    void clrPlot(QCustomPlot *Plot);

private:
    bool COM_state;
    bool COM_state2;

signals:
    void currentCOM(QString);
    void stopCOM();

private slots:
    void onPopUpMenu(const QPoint &mousePosition);
    void slot_showOptionDialog();
    void plotCurrents(QVector<double> currents);
    void getLocData(MatrixXd);

    //select com by button
    void ctrlCOM();
    void handleSendCOM(bool);
    void handleRecvCOM(bool);

};

#endif /* MAGPREDICTORWIDGET_H_ */
