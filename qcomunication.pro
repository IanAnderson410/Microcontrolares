QT       += core gui serialport network quick3d quickwidgets charts

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    qpaintbox.cpp

HEADERS += \
    mainwindow.h \
    qpaintbox.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES += \
    3dmodels/Escena.qml.txt \
    3dmodels/Historial.qml \
    3dmodels/RobotModel.qml \
    3dmodels/main.qml \
    3dmodels/main.qml \
    3dmodels/pitchmodel.qml \
    3dmodels/vistaSuperior.qml \
    lcd-16x2-pinout.jpg

RESOURCES += \
    recursos.qrc
