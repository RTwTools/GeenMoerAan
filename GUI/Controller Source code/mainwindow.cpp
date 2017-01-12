#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <function.h>
#include <QString>
Function *obj;
QString Log;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    obj=new Function();
    ui->detectbtn->setEnabled(false);
    ui->groupBox->setVisible(false);
    ui->startbtn->setVisible(false);
    ui->btnLog->setVisible(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_startbtn_clicked()
{


    if(ui->startbtn->text()=="START SYSTEM"){
        Log="Started System at :";
        Log.append(obj->get_currentTime());

        ui->logmsg->append(Log);
        ui->startbtn->setText("STOP SYSTEM");
        QString str="./scripts_start.sh ";
        str.append(obj->getWorkspace());
        cout<<obj->getWorkspace().toStdString()<<endl;
        obj->start_stop_command(str);
        ui->detectbtn->setEnabled(true);

    }
    else{
        Log="Stopped System at :";
        Log.append(obj->get_currentTime());
        ui->logmsg->append(Log);
        ui->startbtn->setText("START SYSTEM");
        obj->start_stop_command("./scripts_stop.sh ");
        ui->detectbtn->setEnabled(false);
    }

}

void MainWindow::on_detectbtn_clicked()
{


        Log="Holes sent at:";
        Log.append(obj->get_currentTime());
        ui->logmsg->append(Log);
        obj->detect_holes("true");



}

void MainWindow::on_btnLog_clicked()
{
    int size =0;
    if(ui->btnLog->text()==">>")
    {
        ui->btnLog->setText("<<");
        size =this->width()+500;
        QString file=":/scripts_start.sh";
        obj->start_stop_command(file);

    }
    else
    {
        size=this->width()-500;
        ui->btnLog->setText(">>");
    }
    this->window()->resize(size,this->height());
}

void MainWindow::on_wsbtn_clicked()
{
    if(ui->wstxt->toPlainText()!=""){

        obj->setWorkSpace(ui->wstxt->toPlainText());

        ui->wsbtn->setVisible(false);
        ui->wslbl->setVisible(false);
        ui->wstxt->setVisible(false);


        ui->groupBox->setVisible(true);
        ui->startbtn->setVisible(true);
        ui->btnLog->setVisible(true);
        ui->worksp_holder_lbl->setText(obj->getWorkspace());


    }


}
