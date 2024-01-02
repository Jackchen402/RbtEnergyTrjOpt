QT       += core gui network charts

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG(debug, debug|release){
    DEFINES -= QT_NO_DEBUG_OUTPUT
}
else{
    DEFINES += QT_NO_DEBUG_OUTPUT
}
LIBS += -lws2_32
TARGET = gradu_project
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
    drawchart.cpp \
    main.cpp \
    modbus/modbus-data.c \
    modbus/modbus-rtu.c \
    modbus/modbus-tcp.c \
    modbus/modbus.c \
    widget.cpp \
    trajectory.cpp

HEADERS += \
    drawchart.h \
    modbus/config.h \
    modbus/modbus-private.h \
    modbus/modbus-rtu-private.h \
    modbus/modbus-rtu.h \
    modbus/modbus-tcp-private.h \
    modbus/modbus-tcp.h \
    modbus/modbus-version.h \
    modbus/modbus.h \
    widget.h \
    trajectory.h


FORMS += \
        widget.ui

INCLUDEPATH += D:/eigen-3.4.0 \

RESOURCES += \
    image.qrc
