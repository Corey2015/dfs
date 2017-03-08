/**
 * Copyright 2015 Fingerprint Cards AB
 */

#ifndef FPC_LOG_H
#define FPC_LOG_H

#include <cutils/log.h>

#ifdef FPC_DEBUG_LOGGING
#define LOGD(...) ALOGD(__VA_ARGS__)
#else
#define LOGD(...) {}
#endif

#define LOGI(...) ALOGI(__VA_ARGS__)

#define LOGE(...) ALOGE(__VA_ARGS__)

#endif // FPC_LOG_H

