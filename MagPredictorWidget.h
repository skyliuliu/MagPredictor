#ifndef MAGPREDICTORWIDGET_H_
#define MAGPREDICTORWIDGET_H_

#include "glwidget.h"
#include "qcustomplot.h"

#include <QKeyEvent>
#include <QTimer>
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

    int currentSlave;
    enum DataMode{Braw_vs_Bkf, Bkf_vs_Bpre} currentMode;

    MagSensorParams *myParam;
    //MagLocalizer *myLoc;
    MagSerial *mySer;
    MagLocalizerPlus *myLoc;

    void initCustomPlot(QCustomPlot *Plot, int sensorId);
    void init2CustomPlot(QCustomPlot *plot, bool isVms);
    void plotXYZPos(QCustomPlot *Plot, double x, double y, double z);
    void plotData(QCustomPlot *Plot, double in1, double in2);
    void clrPlot(QCustomPlot *Plot);

private:
    enum COMState{connected, disconnected}COM_state;

signals:
    void currentCOM(QString);
    void stopCOM();

private slots:
    void onPopUpMenu(const QPoint &mousePosition);
    void slot_showOptionDialog();

    void getPlotData(MatrixXd rawData, MatrixXd filteredData,
                     MatrixXd caliMagData, MatrixXd predictedData);
    void getLocData(MatrixXd);

    //select com by button
    void ctrlCOM();
    void handleCOMOpen(bool);

    //Slave select
    void selectSlave(int);
    void selectMode(int);
};

#endif /* MAGPREDICTORWIDGET_H_ */
