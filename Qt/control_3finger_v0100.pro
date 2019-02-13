QT -= gui
QT += widgets
QT += network

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH  += E:/WORKSPACES/boost_1_55_0

LIBS += E:/WORKSPACES/boost_1_55_0/stage/x64/lib/libboost_date_time-vc120-mt-gd-1_55.lib \
        E:/WORKSPACES/boost_1_55_0/stage/x64/lib/libboost_thread-vc120-mt-gd-1_55.lib \
        E:/WORKSPACES/boost_1_55_0/stage/x64/lib/libboost_system-vc120-mt-gd-1_55.lib \
        E:/WORKSPACES/boost_1_55_0/stage/x64/lib/libboost_chrono-vc120-mt-gd-1_55.lib \
        E:/WORKSPACES/boost_1_55_0/stage/x64/lib/libboost_regex-vc120-mt-gd-1_55.lib

HEADERS  += AsyncSerial.h \
            BufferedAsyncSerial.h \
            MyRobot_AsyncSerial.h \
            MyRobot_Protocol.h \
            MyRobot_ProtocolDef.h \
            ROBOTIQ3finger_function.h \
            tcpserver.h

SOURCES  += \
            AsyncSerial.cpp \
            BufferedAsyncSerial.cpp \
            MyRobot_AsyncSerial.cpp \
            MyRobot_Protocol.cpp \
            ROBOTIQ3finger_function.cpp \
            tcpserver.cpp \
            main.cpp
