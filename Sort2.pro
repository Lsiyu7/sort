#-------------------------------------------------
#
# Project created by QtCreator 2018-09-10T15:41:08
#
#-------------------------------------------------

QT       += core gui widgets network

TARGET = Sort2
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

SOURCES += auborobot.cpp \
           common.cpp \
           orientation.cpp \
           twist.cpp \
           mainwindow.cpp \
           sortfruit.cpp \
           kinematicmodel.cpp \
           thread_sort.cpp \
           main.cpp

HEADERS += robotics.h \
           auborobot.h \
           common.h \
           orientation.h \
           twist.h \
           auborobot.h \
           robotics.h \
           mainwindow.h \
           sortfruit.h \
           kinematicmodel.h \
           thread_sort.h \
           fjrobot.h

HEADERS += include/CameraClient.h \
           include/CameraCmd.h \
           include/PointCloudTools.h \
           include/sample.h \
           include/ZmqClient.h \
           json/allocator.h \
           json/assertions.h \
           json/config.h \
           json/forwards.h \
           json/json.h \
           json/json_features.h \
           json/json_tool.h \
           json/reader.h \
           json/value.h \
           json/version.h \
           json/writer.h

SOURCES += json/json_reader.cpp \
           json/json_value.cpp \
           json/json_writer.cpp \
           sample/sample1_parameter.cpp \
           sample/sample2_ImgAndCloud.cpp \
           src/CameraClient.cpp \
           src/PointCloudTools.cpp \
           src/ZmqClient.cpp \

FORMS += \
        mainwindow.ui
#********lib select **********************************
unix{
    #32bit OS
    contains(QT_ARCH, i386){

        CONFIG += c++11

        DEFINES += _GLIBCXX_USE_CXX11_ABI=0

        INCLUDEPATH += $$PWD/dependents/robotSDK/inc

        LIBS += $$PWD/dependents/protobuf/linux-x32/lib/libprotobuf.a

        LIBS += $$PWD/dependents/robotController/lib-linux32/libour_alg_i5p.a

        LIBS += -L$$PWD/dependents/log4cplus/linux_x32/lib -llog4cplus

        LIBS += -L$$PWD/dependents/robotSDK/lib/linux_x32/ -lauborobotcontroller

        LIBS += -lpthread

    }
    #64bit OS
    contains(QT_ARCH, x86_64){

        CONFIG += c++11

        INCLUDEPATH += $$PWD/dependents/robotSDK/inc
        INCLUDEPATH += ./include
        INCLUDEPATH += .
        INCLUDEPATH += /home/esir/libzmq/include
        INCLUDEPATH += /usr/include/pcl-1.7

        LIBS += $$PWD/dependents/protobuf/linux-x64/lib/libprotobuf.a

        LIBS += $$PWD/dependents/robotController/lib-linux64/libour_alg_i5p.a

        LIBS += -L$$PWD/dependents/log4cplus/linux_x64/lib -llog4cplus

        LIBS += -L$$PWD/dependents/libconfig/linux_x64/lib/ -lconfig

        LIBS += -L$$PWD/dependents/robotSDK/lib/linux_x64/ -lauborobotcontroller

        LIBS += -lpthread -lz

        LIBS += -L$$PWD/lib/ -lfjrobot

        LIBS += -L/home/esir/libzmq/lib/ -lzmq
        LIBS += -L/home/esir/opencv/opencv-3.4.5/build/lib/ -lopencv_core
        LIBS += -L/home/esir/opencv/opencv-3.4.5/build/lib/ -lopencv_imgcodecs
        LIBS += -L/home/esir/opencv/opencv-3.4.5/build/lib/ -lopencv_highgui
        LIBS += -L/usr/lib/x86_64-linux-gnu/ -ljsoncpp
        LIBS += -L/usr/lib/x86_64-linux-gnu/ -lboost_system
        LIBS += -L/usr/lib/x86_64-linux-gnu/ -lpcl_io
        LIBS += -L/usr/lib/x86_64-linux-gnu/ -lpcl_filters
        LIBS += -L/usr/lib/x86_64-linux-gnu/ -lpcl_common
        LIBS += -L/usr/lib/x86_64-linux-gnu/ -lpcl_visualization

    }
}

## Default rules for deployment.
#qnx: target.path = /tmp/$${TARGET}/bin
#else: unix:!android: target.path = /opt/$${TARGET}/bin
#!isEmpty(target.path): INSTALLS += target

#INCLUDEPATH += $$PWD/auboi5/inc
#DEPENDPATH += $$PWD/auboi5
