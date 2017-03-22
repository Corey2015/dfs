# Copyright (C) 2013 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

LOCAL_PATH := $(call my-dir)

##################################################################################
include $(CLEAR_VARS)
LOCAL_MODULE 		:= libfingerprint_core
LOCAL_SRC_FILES 	:= libs/lib/libfingerprint_core.so
LOCAL_MULTILIB      := 32
LOCAL_MODULE_CLASS 	:= SHARED_LIBRARIES
LOCAL_MODULE_TAGS 	:= optional
LOCAL_MODULE_SUFFIX := .so
include $(BUILD_PREBUILT)

##################################################################################
include $(CLEAR_VARS)
LOCAL_MODULE 		:= libfingerprint_core
LOCAL_MULTILIB      := 64
LOCAL_SRC_FILES 	:= libs/lib64/libfingerprint_core.so
LOCAL_MODULE_CLASS 	:= SHARED_LIBRARIES
LOCAL_MODULE_TAGS 	:= optional
LOCAL_MODULE_SUFFIX := .so
include $(BUILD_PREBUILT)

##################################################################################
include $(CLEAR_VARS)
LOCAL_MODULE := fingerprint.default
LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_C_INCLUDES += \
	$(ANDROID_SYSTEM_ROOT)/hardware/libhardware/include \
	$(ANDROID_SYSTEM_ROOT)/hardware/libhardware_legacy/include/hardware_legacy \
	$(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include  \
	$(TARGET_OUT_HEADERS)/common/inc   \
	$(LOCAL_PATH)/inc \
	$(LOCAL_PATH)/fps_control

LOCAL_SRC_FILES := fingerprint_queue.cpp \
	               fingerprint.cpp             \
	               dfs747_fpsensor.cpp \
				   	 		 fps_analyzer.cpp  \
					 		 	 fps_control/fps_control_linux.c  \
					 		 	 fps_control/fps_control.c    \
								 fps_control/fps_detect_calibration.c   \
								 fps_control/fps_image_calibration.c   \
								 fps_control/debug.c\
								 fps_control/image.c

LOCAL_SHARED_LIBRARIES 	:= 	liblog \
							libcutils \
							libhardware_legacy \
							libfingerprint_core

LOCAL_MODULE_TAGS 		:= optional
include $(BUILD_SHARED_LIBRARY)

##############################################################################
include $(CLEAR_VARS)

LOCAL_SRC_FILES := dfs747b-analyzer.c

LOCAL_MODULE := fpsensor_dbg_B
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(TARGET_OUT_EXECUTABLES)


LOCAL_SHARED_LIBRARIES := liblog \
                          libutils \
                          libcutils

include $(BUILD_EXECUTABLE)
