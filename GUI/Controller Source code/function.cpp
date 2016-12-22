#include "function.h"
#include <QDebug>
#include <qprocess.h>
#include <QStringList>

#include <ctype.h>
#include <sys/types.h>
#include <signal.h>
#include <stdio.h>


Function::Function(QObject *parent) :
    QObject(parent)
{
}

void Function::start_stop_command(QString txt)
{


    cout<<txt.toStdString()<<endl;
    std::string st = txt.toStdString();
    const  char *str=st.c_str();
    cout<<str<<endl;
    system(str);
}

void Function::detect_holes(QString value){
    //    QString v="xterm -hold -e bash -i -c 'rostopic pub  /detect_cmd bolt_detection/Detection  \"detect: ";
    //    v.append(value).append(" \" -1").append("'");
    //   // QString v=QString(" xterm -hold -e bash -i -c   \'").append("rostopic pub -1 /detect_cmd bolt_detection/Detection  \"detect:" ).append(value).append(("\"").append("\'");

    std::string st = value.toStdString();
    const  char *str=st.c_str();

    string command= "./script_hole_detcection.sh ";
    command.append(str);
    command.append(" ");
    command.append(this->getWorkspace().toStdString());
    cout<<command<<endl;
    system(command.c_str());

}

void Function::setWorkSpace(QString txt){

    workspace=txt;
}

QString Function::getWorkspace(){


    return workspace;
}

QString Function::get_currentTime(){

    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);

    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}






































