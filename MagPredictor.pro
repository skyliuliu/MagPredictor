#-------------------------------------------------
#
# Project created by QtCreator 2022-04-13T10:41:43
#
#-------------------------------------------------

QT       += core gui
QT       += opengl serialport printsupport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = MagPredictor
TEMPLATE = app

MOC_DIR         = temp/moc
RCC_DIR         = temp/rcc
UI_DIR          = temp/ui
OBJECTS_DIR     = temp/obj

DESTDIR         = bin
CONFIG         -= debug_and_release

CONFIG += precompile_header
QMAKE_CXXFLAGS += /MP

INCLUDEPATH += $$PWD/3rdparty/glc_lib \
               $$PWD/3rdparty/eigen-3.3.7
LIBS += -lopengl32 -lglu32 -lgdi32 -lDbgHelp

CopyFile = $$PWD/Models/capsule.3DS
CopyFile = $$replace(CopyFile, /, \\)
CopyDir = $$OUT_PWD/bin/models/capsule
CopyDir = $$replace(CopyDir, /, \\)
QMAKE_PRE_LINK += echo d | xcopy /y $$CopyFile $$CopyDir &
message($$QMAKE_PRE_LINK)

include($$PWD/3rdparty/glc_lib/lib/glc_lib.pri)
include($$PWD/MagSerial.pri)
include($$PWD/FilterCpp.pri)
include($$PWD/QCustomPlot.pri)
include($$PWD/MagLocalizer_Algorithm.pri)

win32{
#    INCLUDEPATH += $$PWD/../../3rdparty/QWT3D/include
    INCLUDEPATH += $$PWD/3rdparty/ManifoldUKFAlogorithm
#    LIBS += -L$$PWD/../../3rdparty/QWT3D/lib
#    CONFIG(debug,debug|release){
#        LIBS += -lqwtplot3dd
#    }
#    else{
#        LIBS += -lqwtplot3d
#    }
}

SOURCES += main.cpp\
        MagPredictorWidget.cpp \
    glwidget.cpp

HEADERS  += MagPredictorWidget.h \
    glwidget.h

FORMS    += MagPredictorWidget.ui

RESOURCES += \
    magpredictor.qrc
