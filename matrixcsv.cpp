#include "matrixcsv.h"
using namespace Eigen;
using namespace std;

MatrixCSV::MatrixCSV(QObject *parent) : QObject(parent)
{
    loadPath();
}

void MatrixCSV::loadPath()
{
    fileInfo.path = QDir::currentPath()+"\MagOffset2.csv";

}

void MatrixCSV::writeMatrix(matXdArgConst&mat)
{
    if(!fileInfo.path.isEmpty())
    {
        QFile file(fileInfo.path);
        if(file.open(QIODevice::WriteOnly))
        {
            fileInfo.rows = mat.rows();
            fileInfo.cols = mat.cols();

            for(int i=0; i<fileInfo.rows;i++)
            {
                for(int j=0; j<fileInfo.cols; j++)
                {
                    string data = to_string(mat(i, j));

                    if(j<(fileInfo.cols-1))
                    {
                        data += ",";
                    }

                    file.write(data.c_str());
                }
                file.write("\n");
            }
        qDebug()<<__FUNCTION__<<": file write ok!!";
        }
        else
        {
            qDebug()<<__FUNCTION__<<": file open failed!!";
        }
        file.close();
    }
}

bool MatrixCSV::readMatrix()
{
    if(!fileInfo.path.isEmpty())
    {
        QFile file(fileInfo.path);
        if(file.open(QIODevice::ReadWrite))
        {
            QTextStream stream(&file);
            QStringList matList;
            //matList.clear();
            while(!stream.atEnd())
            {
                matList.push_back(stream.readLine());
            }

            int matRow = matList.size();
            int matCol = matList.at(0).split(",").size();
            MatrixXd mat = MatrixXd::Zero(matRow, matCol);

            for (int row = 0; row < matList.size(); row++)
            {
                QString line = matList.at(row);
                QStringList split = line.split(",");//列数据
                for (int col = 0; col < split.size(); col++)
                {
                    string data = split.at(col).toStdString();
                    mat(row, col)= stof(data);
                }
            }
            emit sendReadMatrix(mat);
            qDebug()<<__FUNCTION__<<"read matrix ok";
            return true;
        }
        else
        {
            return false;
        }
        file.close();
    }
    else
    {
        return false;
    }
}
