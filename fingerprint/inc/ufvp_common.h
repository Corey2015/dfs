#include "stdint.h"

#ifndef __UFVP_COMMON_H
#define __UFVP_COMMON_H
typedef struct {
    const char *name;
    uint32_t module_version;
    uint32_t api_version;
    const char *author;
}ufvp_module_t;
#endif

