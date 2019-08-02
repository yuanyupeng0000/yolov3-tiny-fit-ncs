DISTFILES += \
    ../README.md \
    ../CMakeLists.txt

HEADERS += \
    ../Common.h \
    ../object_detection_demo_yolov3_async.hpp

SOURCES += \
    ../Common.cpp \
    ../main.cpp
INCLUDEPATH += /opt/intel/computer_vision_sdk/deployment_tools/inference_engine/include \
    /opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/common \
    /opt/intel/computer_vision_sdk/deployment_tools/inference_engine/src/extension
