QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    common_lib/association.cc \
    kalman_tracker/kalman_base.cc \
    main.cpp \
    mainwindow.cpp \
    qcustomplot.cpp \
    simple_tracker/simple_tracker.cc

HEADERS += \
    common_lib/association.h \
    kalman_tracker/kalman_base.h \
    mainwindow.h \
    qcustomplot.h \
    simple_tracker/simple_tracker.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

