#-------------------------------------------------
#
# Project created by QtCreator 2012-12-08T16:53:56
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = robotdriver
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    robotdriver.cpp

include(qextserialport/qextserialport.pri)

HEADERS += \
    robotdriver.h

#DEFINES += QT_NO_DEBUG_OUTPUT
