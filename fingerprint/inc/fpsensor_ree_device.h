/*
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 */

#ifndef FPC_REE_DEVICE_H
#define FPC_REE_DEVICE_H

#include <stdbool.h>

typedef struct fpc_ree_device fpc_ree_device_t;

void fpc_ree_device_destroy(fpc_ree_device_t* device);
fpc_ree_device_t* fpc_ree_device_new();


int fpc_ree_device_wait_irq(fpc_ree_device_t* device);
int fpc_ree_device_set_cancel(fpc_ree_device_t* device);
int fpc_ree_device_clear_cancel(fpc_ree_device_t* device);

int fpc_ree_device_set_clock_enabled(fpc_ree_device_t* device, bool enabled);
int fpc_ree_device_set_ttw_enabled(fpc_ree_device_t* device, bool enabled);

#endif // FPC_REE_DEVICE_H
