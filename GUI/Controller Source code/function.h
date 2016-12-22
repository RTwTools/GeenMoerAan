#ifndef FUNCTION_H
#define FUNCTION_H

#include<iostream>
#include <QObject>
#include <string>
using namespace std;


class Function : public QObject
{
    Q_OBJECT
private:
    QString workspace;
public:
    explicit Function(QObject *parent = 0);
    Q_INVOKABLE void start_stop_command(QString txt);
    Q_INVOKABLE void detect_holes(QString value);

    Q_INVOKABLE void setWorkSpace(QString txt);
    Q_INVOKABLE QString getWorkspace();
    Q_INVOKABLE QString get_currentTime();

signals:

public slots:

};

#endif
