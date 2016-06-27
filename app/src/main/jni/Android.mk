# OpenCV path setting

LOCAL_PATH := $(call my-dir)
OPENCV_SDK_PATH := D:\Users\Mark\OpenCV-android-sdk
OPENCV_INC_PATH := $(OPENCV_SDK_PATH)/sdk/native/jni/include
#OPENCV_SO_PATH := $(OPENCV_SDK_PATH)/sdk/native/libs/armeabi-v7a



# Specify OpenCV.so location




# Build our library


OpenCV_INSTALL_MODULES := on
OpenCV_CAMERA_MODULES := off

OPENCV_LIB_TYPE := STATIC


include $(CLEAR_VARS)
include $(OPENCV_SDK_PATH)/sdk/native/jni/OpenCV.mk

LOCAL_MODULE := myJNI
LOCAL_CPPFLAGS := -std=c++11
LOCAL_LDLIBS     += -llog -ldl

LOCAL_SRC_FILES :=myJNI.cpp
LOCAL_SRC_FILES += \
    += EDLineDetector.cpp \
    += categorization_line_segment.cpp \
    += information_matrix_decay.cpp \
    += information_matrix_from_ls_2.cpp \
    += line_segment.cpp \
    += line_segment_critical.cpp \
    += line_segment_normal_form.cpp \
    += line_segment_valid_2.cpp \
    += transformation_coordinate.cpp \
    += vanishing_point_ransac.cpp \
    += vanishing_point_module.cpp

LOCAL_C_INCLUDES += $(LOCAL_PATH) \
                    $(OPENCV_INC_PATH)





include $(BUILD_SHARED_LIBRARY)