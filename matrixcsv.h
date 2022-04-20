#ifndef MATRIXCSV_H
#define MATRIXCSV_H

#include <QObject>
#include <QString>
#include <QFileDialog>
#include <QMessageBox>
#include <string>
#include <iostream>
#include <QFileDialog>
#include <Eigen/Dense>
#include <QDir>
#include <fstream>
#include <sstream>
#include <QDebug>
#include "filtercommon.h"

class MatrixCSV : public QObject
{
    Q_OBJECT
public:
    explicit MatrixCSV(QObject *parent = 0);
    bool readMatrix();
    void loadPath();
    void writeMatrix(matXdArgConst&mat);
    QString readPath()const
    {return fileInfo.path;}

private:
    struct
    {
        QString path;
        int rows;
        int cols;
        QStringList allData;
    }fileInfo;

signals:
    void sendReadMatrix(MatrixXd);

public slots:

};

#endif // MATRIXCSV_H
