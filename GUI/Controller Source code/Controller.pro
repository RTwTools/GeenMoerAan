#-------------------------------------------------
#
# Project created by QtCreator 2016-12-09T05:20:08
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Controller
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    function.cpp

HEADERS  += mainwindow.h \
    function.h

FORMS    += mainwindow.ui

RESOURCES += \
    images.qrc \
    scripts.qrc
