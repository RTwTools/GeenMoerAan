/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QTextEdit>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *startbtn;
    QGroupBox *groupBox;
    QPushButton *detectbtn;
    QLabel *label;
    QLabel *label_2;
    QPushButton *btnLog;
    QLabel *wslbl;
    QTextEdit *wstxt;
    QPushButton *wsbtn;
    QLabel *wslbl_2;
    QLabel *worksp_holder_lbl;
    QTextEdit *logmsg;
    QLabel *label_3;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(502, 435);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        startbtn = new QPushButton(centralWidget);
        startbtn->setObjectName(QString::fromUtf8("startbtn"));
        startbtn->setGeometry(QRect(140, 30, 171, 81));
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(20, 200, 441, 131));
        detectbtn = new QPushButton(groupBox);
        detectbtn->setObjectName(QString::fromUtf8("detectbtn"));
        detectbtn->setGeometry(QRect(10, 40, 121, 71));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 350, 21, 21));
        label->setAutoFillBackground(false);
        label->setPixmap(QPixmap(QString::fromUtf8(":/copyright/copyright.png")));
        label->setScaledContents(true);
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(30, 360, 101, 21));
        QFont font;
        font.setPointSize(9);
        font.setBold(true);
        font.setWeight(75);
        label_2->setFont(font);
        label_2->setTextFormat(Qt::PlainText);
        btnLog = new QPushButton(centralWidget);
        btnLog->setObjectName(QString::fromUtf8("btnLog"));
        btnLog->setGeometry(QRect(410, 10, 81, 27));
        wslbl = new QLabel(centralWidget);
        wslbl->setObjectName(QString::fromUtf8("wslbl"));
        wslbl->setGeometry(QRect(20, 150, 171, 20));
        wstxt = new QTextEdit(centralWidget);
        wstxt->setObjectName(QString::fromUtf8("wstxt"));
        wstxt->setGeometry(QRect(200, 140, 201, 31));
        wstxt->setInputMethodHints(Qt::ImhNone);
        wstxt->setTabChangesFocus(true);
        wstxt->setLineWrapMode(QTextEdit::NoWrap);
        wstxt->setAcceptRichText(true);
        wsbtn = new QPushButton(centralWidget);
        wsbtn->setObjectName(QString::fromUtf8("wsbtn"));
        wsbtn->setGeometry(QRect(410, 140, 81, 31));
        wslbl_2 = new QLabel(centralWidget);
        wslbl_2->setObjectName(QString::fromUtf8("wslbl_2"));
        wslbl_2->setGeometry(QRect(10, 0, 121, 20));
        worksp_holder_lbl = new QLabel(centralWidget);
        worksp_holder_lbl->setObjectName(QString::fromUtf8("worksp_holder_lbl"));
        worksp_holder_lbl->setGeometry(QRect(130, 0, 121, 20));
        logmsg = new QTextEdit(centralWidget);
        logmsg->setObjectName(QString::fromUtf8("logmsg"));
        logmsg->setGeometry(QRect(540, 30, 421, 321));
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(540, 10, 67, 17));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 502, 25));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        startbtn->setText(QApplication::translate("MainWindow", "START SYSTEM", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MainWindow", "Funtions", 0, QApplication::UnicodeUTF8));
        detectbtn->setText(QApplication::translate("MainWindow", "Start Detection", 0, QApplication::UnicodeUTF8));
        label->setText(QString());
        label_2->setText(QApplication::translate("MainWindow", "Geenmoeraan", 0, QApplication::UnicodeUTF8));
        btnLog->setText(QApplication::translate("MainWindow", ">>", 0, QApplication::UnicodeUTF8));
        wslbl->setText(QApplication::translate("MainWindow", "Enter Workspace Name:", 0, QApplication::UnicodeUTF8));
        wsbtn->setText(QApplication::translate("MainWindow", "Enter", 0, QApplication::UnicodeUTF8));
        wslbl_2->setText(QApplication::translate("MainWindow", "Your workspace:", 0, QApplication::UnicodeUTF8));
        worksp_holder_lbl->setText(QString());
        label_3->setText(QApplication::translate("MainWindow", "<html><head/><body><p>Logs</p><p><br/></p></body></html>", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
