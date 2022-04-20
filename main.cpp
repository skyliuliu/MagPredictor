#include "MagPredictorWidget.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MagPredictorWidget w;
    w.show();

    return a.exec();
}
