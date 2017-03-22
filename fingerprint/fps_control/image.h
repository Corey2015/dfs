#ifndef __image_h__
#define __image_h__


#include <stdint.h>
#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
//
// Prototypes
//

int save_bmp(__INPUT char_t  *path,
             __INPUT uint8_t *img,
             __INPUT size_t   size);

#ifdef __cplusplus
}
#endif


#endif // __image_h__
