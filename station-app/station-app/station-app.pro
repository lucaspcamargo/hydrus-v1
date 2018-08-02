#-------------------------------------------------
#
# Project created by QtCreator 2016-05-19T19:54:48
#
#-------------------------------------------------

QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
LIBS    += -lmarblewidget-qt5 -lKF5CoreAddons
INCLUDEPATH += /usr/include/KF5/KCoreAddons

TARGET = station-app
TEMPLATE = app

CONFIG += c++11

SOURCES += main.cpp\
        mainwindow.cpp \
    statuswidget.cpp \
    rcwidget.cpp \
    droneconnection.cpp \
    serverconfigdialog.cpp \
    aboutdialog.cpp \
    dronemap.cpp \
    droneroute.cpp \
#    dwcontrollerhub.cpp \
#    dwcontrollerstate.cpp
    thirdparty/joystick.cpp \
    dronerouteset.cpp \
    dronesimulator.cpp \
    dronelog.cpp \
    logtimelinewidget.cpp

HEADERS  += mainwindow.h \
    statuswidget.h \
    rcwidget.h \
    droneconnection.h \
    droneblackboard.h \
    serverconfigdialog.h \
    aboutdialog.h \
    dronemap.h \
    droneroute.h \
#    dwcontrollerhub.h \
#    dwcontrollerstate.h
    thirdparty/joystick.h \
    dronerouteset.h \
    dronesimulator.h \
    dronelog.h \
    logtimelinewidget.h

FORMS    += mainwindow.ui \
    statuswidget.ui \
    rcwidget.ui \
    serverconfigdialog.ui \
    aboutdialog.ui \
    logtimelinewidget.ui

RESOURCES += \
    resources.qrc


unix{
    #CONFIG += link_pkgconfig
    #PKGCONFIG += sdl2
    #DEFINES+="DW_USE_SDL2=1"
}
