#include "qserialcombobox.h"
#include <QDebug>

QSerialComboBox::QSerialComboBox(QWidget *parent):QComboBox(parent)
{

}

void QSerialComboBox::updateListItems(QStringList items){
    clear();
    addItems(items);

}
