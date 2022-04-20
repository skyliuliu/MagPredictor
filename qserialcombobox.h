#ifndef QSERIALCOMBOBOX_H
#define QSERIALCOMBOBOX_H

#include <QObject>
#include <QComboBox>

class QSerialComboBox : public QComboBox
{
    Q_OBJECT
public:
    explicit QSerialComboBox(QWidget *parent = nullptr);

public slots:
    void updateListItems(QStringList);

signals:
    void showList();
};

#endif // QSERIALCOMBOBOX_H
