TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp
win32 { 
QMAKE_LFLAGS += -static -static-libgcc -static-libstdc++ 
}
