/*
 * Copyright (C) 2014 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "FingerprintHal"

#include <errno.h>
#include <string.h>
#include <cutils/log.h>
#include <hardware/hardware.h>
#include <hardware/fingerprint.h>
#include <hardware_legacy/power.h>
#include <hardware_legacy/vibrator.h>
#include <cutils/properties.h>
#include "fpsensor_l.h"
#include <linux/fb.h>
#include "fpsensor.h"
#include "fps_driver.h"


//#define ENABLE_DUPLICATE_CHECK
#define FPC_REQUIRED_SAMPLES_ENROL 10

#define IDLE_TIMEOUT 3
#define RECOVERY_MAX_COUNT 10

#define DEFATULT_TIMEOUT 60
#define NO_ERROR 0

#define TEMPLATE_PATH                		"/data/system/users"


int32_t dfs747_device_init(fingerprint_data_t *device);
static int fingerprint_cmd(fingerprint_data_t *fpData,fingerprintQueue *queue,fingerprint_cmdID_t cmdId);


static int fingerprint_end_verify(fingerprint_data_t *fpData,fingerprint_msg_t *msg);

int32_t gselftest_result = -1;

void set_wakelock_state(fingerprint_data_t *fpData, bool acquire)
{
    static bool mWakelock_aquire = false;
    struct dfs747_ioc_transfer tr;
    if(acquire) {
        if(!mWakelock_aquire) {
            mWakelock_aquire = true;
            tr.len    = 1;
            tr.opcode = DFS747_IOC_WAKELOCK;
            ioctl(fpData->sysfs_fd, DFS747_IOC_MESSAGE(1), &tr);
            ALOGD("%s: acquire %d\n", __func__,acquire);
        }
    }else {
        if(mWakelock_aquire) {
            mWakelock_aquire = false;
            tr.len    = 0;
            tr.opcode = DFS747_IOC_WAKELOCK;
            ioctl(fpData->sysfs_fd, DFS747_IOC_MESSAGE(1), &tr);
            ALOGD("%s: release %d\n", __func__,acquire);
        }
    }
}

static uint64_t get_64bit_rand() {
    // This should use a cryptographically-secure random number generator like arc4random().
    // It should be generated inside of the TEE where possible. Here we just use something
    // very simple.
    ALOGD("----------------> %s ----------------->", __FUNCTION__);
    uint64_t r = (((uint64_t)rand()) << 32) | ((uint64_t)rand());
    return r != 0 ? r : 1;
}

int32_t fingerprint_exit(hw_device_t *dev)
{
    int32_t rc = NO_ERROR;
    fingerprint_data_t *fpData = (fingerprint_data_t *)dev;
    if (fpData->cmd_pid == 0) {
        return rc;
    }

    fingerprint_cmd_t *node = (fingerprint_cmd_t *)malloc(sizeof(fingerprint_cmd_t));
    if (NULL == node) {
        ALOGE("%s: No memory for fingerprint_cmd_t", __func__);
        return -ENOMEM;
    }

    memset(node, 0, sizeof(fingerprint_cmd_t));
    node->cmdId= FINGERPRINT_CMD_EXIT;

    if (fpData->cmd_queue->enqueue((void *)node)) {
        fingerprint_sem_post(&fpData->cmd_sem);
    } else {
        free(node);
		node = NULL;
    }
    /* wait until cmd thread exits */
    if (pthread_join(fpData->cmd_pid, NULL) != 0) {
        ALOGD("%s: cmd_thread dead already\n", __func__);
    }
    fpData->cmd_pid = 0;

    if (pthread_join(fpData->poll_pid, NULL) != 0) {
        ALOGD("%s: poll_thread dead already\n", __func__);
    }
    fpData->poll_pid = 0;

    return rc;
}

static int fingerprint_close(hw_device_t *dev)
{
    int ret = 0;

    if (dev == NULL) {
        ALOGE("device already closed");
        return 0;
    }

    fingerprint_data_t *fpData = (fingerprint_data_t *)dev;

    fingerprint_exit(dev);

    fingerprint_sem_destroy(&fpData->cmd_sem);
	
    fpData->sleep(fpData,1);
	free(fpData->tac_handle);

    delete  fpData->cmd_queue;
    delete  fpData->evt_queue;

    if (fpData)
        free(fpData);

    return 0;
}

static int fingerprint_cmd(fingerprint_data_t *fpData,fingerprintQueue *queue,fingerprint_cmdID_t cmdId)
{
    fingerprint_cmd_t *node = (fingerprint_cmd_t *)malloc(sizeof(fingerprint_cmd_t));
    if (NULL == node) {
        ALOGE("%s: No memory for fingerprint_cmd_t", __func__);
        return -ENOMEM;
    }
    memset(node, 0, sizeof(fingerprint_cmd_t));
    node->cmdId= cmdId;

    if (queue->enqueue((void *)node) == true) {
        fingerprint_sem_post(&fpData->cmd_sem);
        return NO_ERROR;
    } else {
        ALOGE("cmdId is %d,free because of enqueque fail",node->cmdId);
        free(node);
		node = NULL;
        return FINGERPRINT_ERROR;
    }
}

static int fingerprint_cmd_wait(fingerprint_data_t *fpData,fingerprint_cmdID_t cmdId,uint64_t data,uint64_t *out)
{
    int ret = 0;
    int delta_us = 0;

    struct timeval ts_start;
    struct timeval ts_current;
    struct timeval ts_delta;
    fingerprint_cmd_t *node = (fingerprint_cmd_t *)malloc(sizeof(fingerprint_cmd_t));
    if (NULL == node) {
        ALOGE("%s: No memory for fingerprint_cmd_t", __func__);
        return -ENOMEM;
    }

    gettimeofday(&ts_start, NULL);
    memset(node, 0, sizeof(fingerprint_cmd_t));
    node->cmdId= cmdId;
    node->data= data;	
    node->reserved_node = true;	

    if (fpData->cmd_queue->enqueue((void *)node) == true) {
        fingerprint_sem_post(&fpData->cmd_sem);
        ret = fingerprint_sem_wait(&fpData->sync_sem);
        if (ret != 0 && errno != EINVAL) {
               ALOGE("%s: fingprint_sem_wait error (%s)",
                           __func__, strerror(errno));
        }
        ret = node->status;
	 if(out)
		*out = node->data;
	 
        free(node);
        node = NULL;
        gettimeofday(&ts_current, NULL);
        timersub(&ts_current, &ts_start, &ts_delta);
        delta_us = TIME_IN_US(ts_delta);
        ALOGD("KPI do cmdId %d : %d ms",cmdId, delta_us / 1000);
        return ret; 
    } else {
        free(node);
		node = NULL;
        return FINGERPRINT_ERROR;
    }
}

static uint64_t fingerprint_pre_enroll(struct fingerprint_device *device)
{
    uint64_t challenge = 0;
    ALOGD("%s",__func__);
    fingerprint_data_t *fpData  = (fingerprint_data_t *)device;
    fingerprint_cmd_wait(fpData,FINGERPRINT_CMD_PRE_ENROL,0,&challenge);
	return challenge;
}

static int fingerprint_post_enroll(struct fingerprint_device *device)
{
    uint64_t challenge = 0;

    ALOGD("%s",__func__);
    fingerprint_data_t *fpData  = (fingerprint_data_t *)device;
    fingerprint_cmd_wait(fpData,FINGERPRINT_CMD_POST_ENROL,0,&challenge);
	return challenge;
}

static uint64_t fingerprint_get_auth_id(struct fingerprint_device *device)
{
    uint64_t auth_id;
    ALOGD("%s",__func__);
    fingerprint_data_t *fpData  = (fingerprint_data_t *)device;
    fingerprint_cmd_wait(fpData,FINGERPRINT_CMD_GET_AUTH_ID,0,&auth_id);
	return auth_id;
}

static int fingerprint_set_active_group(struct fingerprint_device *dev, uint32_t gid,
        const char *path) 
{
    fingerprint_data_t *fpData  = (fingerprint_data_t *)dev;
    int ret = 0;
	uint32_t i;
    int delta_us = 0;
    struct timeval ts_start;
    struct timeval ts_current;
    struct timeval ts_delta;

    ALOGD("%s",__func__);
    fingerprint_cmd_t *node = (fingerprint_cmd_t *)malloc(sizeof(fingerprint_cmd_t));
    if (NULL == node) {
        ALOGE("%s: No memory for fingerprint_cmd_t", __func__);
        return -ENOMEM;
    }

    gettimeofday(&ts_start, NULL);
    
    memset(node, 0, sizeof(fingerprint_cmd_t));

    node->cmdId= FINGERPRINT_CMD_SET_ACTIVE_GRP;
    node->data = (uint64_t)gid;
    node->buf = (char *)path;
    node->reserved_node = true;
	
    if (fpData->cmd_queue->enqueue((void *)node) == true) {
        fingerprint_sem_post(&fpData->cmd_sem);
        ret = fingerprint_sem_wait(&fpData->sync_sem);
           if (ret != 0 && errno != EINVAL) {
               ALOGE("%s: fingprint_sem_wait error (%s)",
                           __func__, strerror(errno));
		}

     ret = node->status;
	 ALOGD("calibrate result is %s",ret);
	 free(node);
	 node = NULL;
	 gettimeofday(&ts_current, NULL);
	 timersub(&ts_current, &ts_start, &ts_delta);
	 delta_us = TIME_IN_US(ts_delta);
	 ALOGD("KPI fingerprint_calibrate: %d ms",delta_us / 1000);
        return ret;
    } else {
        free(node);
		node = NULL;
        return FINGERPRINT_ERROR;
    }
}

static int fingerprint_calibrate(struct fingerprint_device *dev, const char *store_path)
{
    fingerprint_data_t *fpData  = (fingerprint_data_t *)dev;
    int ret = 0;
	uint32_t i;
    int delta_us = 0;
    struct timeval ts_start;
    struct timeval ts_current;
    struct timeval ts_delta;

    fingerprint_cmd_t *node = (fingerprint_cmd_t *)malloc(sizeof(fingerprint_cmd_t));
    if (NULL == node) {
        ALOGE("%s: No memory for fingerprint_cmd_t", __func__);
        return -ENOMEM;
    }

    gettimeofday(&ts_start, NULL);
    
    memset(node, 0, sizeof(fingerprint_cmd_t));

    node->cmdId= FINGERPRINT_CMD_CALIBRATE;
    node->buf = (char *)store_path;
    node->reserved_node = true;
	
    if (fpData->cmd_queue->enqueue((void *)node) == true) {
        fingerprint_sem_post(&fpData->cmd_sem);
        ret = fingerprint_sem_wait(&fpData->sync_sem);
           if (ret != 0 && errno != EINVAL) {
               ALOGE("%s: fingprint_sem_wait error (%s)",
                           __func__, strerror(errno));
		}

     ret = node->status;
	 ALOGD("calibrate result is %s",ret);
	 free(node);
	 node = NULL;
	 gettimeofday(&ts_current, NULL);
	 timersub(&ts_current, &ts_start, &ts_delta);
	 delta_us = TIME_IN_US(ts_delta);
	 ALOGD("KPI fingerprint_calibrate: %d ms",delta_us / 1000);
        return ret;
    } else {
        free(node);
		node = NULL;
        return FINGERPRINT_ERROR;
    }
}

static int fingerprint_enroll(struct fingerprint_device *device,
        const hw_auth_token_t *hat, uint32_t gid, uint32_t timeout_sec) 
{
    ALOGD("fingerprint_enroll");
    fingerprint_data_t* dev = (fingerprint_data_t*) device;
    fingerprint_data_t *fpData  = (fingerprint_data_t *)dev;

    // TODO: store enrolled fingerprints, authenticator id, and secure_user_id
    fingerprint_cmd_t *node = (fingerprint_cmd_t *)malloc(sizeof(fingerprint_cmd_t));
    if (NULL == node) {
        ALOGE("%s: No memory for fingerprint_cmd_t", __func__);
        return -ENOMEM;
    }

    if(timeout_sec < 180)
        timeout_sec = 180;

    memset(node, 0, sizeof(fingerprint_cmd_t));
    node->cmdId= FINGERPRINT_CMD_ENROLL;
    node->data = timeout_sec;
    node->len = gid;
    node->buf = (char *)hat;

    if (fpData->cmd_queue->enqueue((void *)node) == true) {
        fingerprint_sem_post(&fpData->cmd_sem);
        return NO_ERROR;
    } else {
        free(node);
		node = NULL;
        return FINGERPRINT_ERROR;
    }	
    return 0;

}

static int fingerprint_cancel(struct fingerprint_device *device) {
    ALOGD("fingerprint_cancel");
    fingerprint_data_t *fpData  = (fingerprint_data_t *)device;
    return fingerprint_cmd(fpData,fpData->cmd_queue,FINGERPRINT_CMD_CANCEL);
}

static int fingerprint_authenticate(struct fingerprint_device *device,
    uint64_t operation_id, uint32_t gid)
{
    ALOGD("fingerprint_authenticate");
    fingerprint_data_t *fpData  = (fingerprint_data_t *)device;
    int rc = 0;
    unsigned int i = 0;
    unsigned int j = 0;
	
    if (!device->notify) {
        ALOGE("%s failed notify not set\n", __func__);
        rc = -1;
        return -1;
    }
		
    fingerprint_cmd_t *node = (fingerprint_cmd_t *)malloc(sizeof(fingerprint_cmd_t));
    if (NULL == node) {
        ALOGE("%s: No memory for fingerprint_cmd_t", __func__);
        return -ENOMEM;
    }
    memset(node, 0, sizeof(fingerprint_cmd_t));

    node->cmdId = FINGERPRINT_CMD_VERIFY;
    node->data = operation_id;
    node->len = gid;

    if (fpData->cmd_queue->enqueue((void *)node) == true) {
        fingerprint_sem_post(&fpData->cmd_sem);
        return NO_ERROR;
    }

err:
    free(node);
	node = NULL;
    return FINGERPRINT_ERROR;
}


static int fingerprint_remove(struct fingerprint_device __unused *dev,
        uint32_t __unused gid, uint32_t __unused fid) 
{
    // TODO: implement enroll and remove, and set dev->authenticator_id = 0 when no FPs enrolled

    fingerprint_data_t *fpData  = (fingerprint_data_t *)dev;

    fingerprint_cmd_t *node = (fingerprint_cmd_t *)malloc(sizeof(fingerprint_cmd_t));
    if (NULL == node) {
        ALOGE("%s: No memory for fingerprint_cmd_t", __func__);
        return -ENOMEM;
    }

    memset(node, 0, sizeof(fingerprint_cmd_t));
    node->cmdId= FINGERPRINT_CMD_REMOVE;
    node->len = gid;
    node->data= (uint64_t)fid;
    if (fpData->cmd_queue->enqueue((void *)node) == true) {
        fingerprint_sem_post(&fpData->cmd_sem);
        return NO_ERROR;
    } else {
        free(node);
		node = NULL;
        return FINGERPRINT_ERROR;
    }
    return FINGERPRINT_ERROR;
}


static int fingerprint_enumerate(struct fingerprint_device *device,
        fingerprint_finger_id_t *results, uint32_t *max_size) 
{
    fingerprint_data_t *fpData  = (fingerprint_data_t *)device;
    int ret = 0;
    int delta_us = 0;

    struct timeval ts_start;
    struct timeval ts_current;
    struct timeval ts_delta;
    fingerprint_cmd_t *node = (fingerprint_cmd_t *)malloc(sizeof(fingerprint_cmd_t));
    if (NULL == node) {
        ALOGE("%s: No memory for fingerprint_cmd_t", __func__);
        return -ENOMEM;
    }

    gettimeofday(&ts_start, NULL);
    memset(node, 0, sizeof(fingerprint_cmd_t));
    node->cmdId= FINGERPRINT_CMD_ENUMERATE;
    node->len= *max_size;	
    node->buf= (char *)results;	
    node->reserved_node = true;	

    if (fpData->cmd_queue->enqueue((void *)node) == true) {
        fingerprint_sem_post(&fpData->cmd_sem);
        ret = fingerprint_sem_wait(&fpData->sync_sem);
        if (ret != 0 && errno != EINVAL) {
               ALOGE("%s: fingprint_sem_wait error (%s)",
                           __func__, strerror(errno));
        }
        ret = node->status;
	    *max_size = node->len;
	 
        free(node);
        node = NULL;
        gettimeofday(&ts_current, NULL);
        timersub(&ts_current, &ts_start, &ts_delta);
        delta_us = TIME_IN_US(ts_delta);
        ALOGD("KPI do enum : %d ms", delta_us / 1000);
        return ret; 
    } else {
        free(node);
		node = NULL;
        return FINGERPRINT_ERROR;
    }
}

static int fingerprint_do_enumerate(struct fingerprint_device *device,
        fingerprint_finger_id_t *results, uint32_t *max_size) 
{
    ALOGD("%s", __func__);
    fingerprint_data_t *fpData  = (fingerprint_data_t *)device;


  int retval;
  size_t templateID;
    int i;
   uint32_t size = 0;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) fpData->tac_handle;

  for(i=0;(i<MAX_NBR_TEMPLATES) && (size < *max_size);i++)
    {
        if((tac_handle->template_state[i] == 0xaa) && (tac_handle->pFeature_data[i]))
        {
            results[size].fid = i;
            results[size].gid = fpData->current_gid;
            size++;
        }
    }
    *max_size = size;
    
    ALOGD("%s template_count %u\n", __func__, size);

    return 0;
}


static int set_notify_callback(struct fingerprint_device *dev,
                                fingerprint_notify_t notify) {
    /* Decorate with locks */
    dev->notify = notify;
    ALOGE("set notify");
    return 0;
}

bool fingerprint_match_cmdId(void *data, int match_data)
{
  bool rc = false;
  fingerprint_cmd_t * cmd = (fingerprint_cmd_t *) data;
  if(cmd == NULL) {
      ALOGE("cmd is NULL");
      return false;
  }
  rc = cmd->cmdId == match_data;
  if(rc)
        ALOGE("del cmd %d success",cmd->cmdId);
  return rc;
}

static int fingerprint_do_remove(fingerprint_device_t *device,uint32_t gid, uint32_t fid)
{
    fingerprint_data_t *fpData  = (fingerprint_data_t *)device;
    fingerprint_msg_t msg;
    int status = 0;

    uint32_t indices_count = 0;
    uint32_t* indices = NULL;
    int found = 0; // true if at least one template was found

    ALOGD("%s(fid=%d gid=%d)", __func__, fid, gid);

    if (gid != fpData->current_gid) {
        ALOGD("%s gid != current_gid, nothing to remove\n", __func__);
        goto out;
    }


    for (uint32_t i = 1; i <= MAX_NBR_TEMPLATES; i++) {

        if (fid == 0 || fid == i) {
            if (fpData->delete_template(fpData->tac_handle, i)) {
                status = -EIO;
                goto out;
            }
			ALOGD("notify removed fid=%d gid=%d", fid, fpData->current_gid);
			msg.type = FINGERPRINT_TEMPLATE_REMOVED;
			msg.data.removed.finger.fid = fid;
			msg.data.removed.finger.gid = fpData->current_gid;
			device->notify(&msg);
            found = 1;
        }
    }

    if (!found && fid != 0) {
        // Fingerprint not found in the database, notify it was already removed by sending
        // FINGERPRINT_TEMPLATE_REMOVED.
        ALOGD("fingerprint not found, notifying removed fid=%d gid=%d", fid, fpData->current_gid);
        if (device->notify) {
            msg.type = FINGERPRINT_TEMPLATE_REMOVED;
            msg.data.removed.finger.fid = fid;
            msg.data.removed.finger.gid = fpData->current_gid;
            device->notify(&msg);
        }
    }

out:
    if (status) {
        msg.type = FINGERPRINT_ERROR;
        msg.data.error = FINGERPRINT_ERROR_UNABLE_TO_PROCESS;
        device->notify(&msg);
	}

    return 0;
}


int fingerprint_enroll_init(fingerprint_data_t *fpData, const hw_auth_token_t *hat, uint32_t  gid,uint32_t  timeout_sec) 
{
    fingerprint_device_t *device = (fingerprint_device_t *)fpData;
    fingerprint_msg_t msg;
    int ret = 0;
    int status = 0;
    uint32_t indices_count = 0;

    ALOGE("do enrol init:timeout =%d",timeout_sec);
	if(fpData->image_mode == 0) {
        ALOGD("%s image mode not enable,can't enroll\n", __func__);
        status = -EPERM;
        goto out;
    }	

    status = fpData->begin_enroll(fpData->tac_handle);
    if (status != FPSENSOR_ERROR_OK) {
        status = -EIO;
        goto out;
    }

	
    gettimeofday(&fpData->enrol_timeout, NULL);
    fpData->finger_state = FINGERPRINT_STATE_ENROLLING;
    fpData->enroll_remaining = 10;

    if(timeout_sec!=0){
            fpData->enrol_timeout.tv_sec  += timeout_sec;
    }else{
        fpData->enrol_timeout.tv_sec  += DEFATULT_TIMEOUT;
    }
    
    if(fpData->state_notified == 0) {
		msg.type = FINGERPRINT_ACQUIRED;
		msg.data.acquired.acquired_info = FINGERPRINT_ACQUIRED_WAIT_FINGERDOWN;
		device->notify(&msg);	
	}
    if(fpData->finger_press_state == FINGERPRINT_WAIT_FINGERUP)
        fpData->finger_press_state = FINGERPRINT_WAIT_FINGERDOWN;
	fpData->sleep(fpData,!fpData->touch_mode);
    return 0;
	
out:
    switch (status) {
    case 0:
        ALOGD("%s completed\n", __func__);
        break;
	case -EPERM:
        ALOGI("%s image_mode not enable\n", __func__);
        msg.data.error = FINGERPRINT_ERROR_NOT_ENABLED;
        msg.type = FINGERPRINT_ERROR;
        device->notify(&msg);
		break;
    case -EINTR:
        ALOGI("%s cancelled\n", __func__);
        msg.data.error = FINGERPRINT_ERROR_CANCELED;
        msg.type = FINGERPRINT_ERROR;
        device->notify(&msg);
        break;
    case -ETIMEDOUT:
        ALOGD("%s timed out\n", __func__);
        msg.data.error = FINGERPRINT_ERROR_TIMEOUT;
        msg.type = FINGERPRINT_ERROR;
        device->notify(&msg);
        break;
    case -ENOMEM:
	    ALOGD("%s no space to enroll\n", __func__);
        msg.data.error = FINGERPRINT_ERROR_UNABLE_TO_PROCESS;
        msg.type = FINGERPRINT_ERROR;
        device->notify(&msg);
	  break;
    default:
        ALOGE("%s failed %i\n", __func__, status);
        msg.data.error = FINGERPRINT_ERROR_HW_UNAVAILABLE;
        msg.type = FINGERPRINT_ERROR;
        device->notify(&msg);
        break;
    }

    return status;
}

int fingerprint_end_enroll(fingerprint_data_t *fpData,fingerprint_msg_t *msg)
{
    fingerprint_device_t *device = (fingerprint_device_t *)fpData;
    fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*)fpData->tac_handle;
	if(msg)
	    device->notify(msg);
	if(tac_handle->enrolling == false)
	{
		ALOGD("enroll already end before");
		return 0;
	}
    fpData->finger_state = FINGERPRINT_STATE_LISTENING;
    fpData->enroll_remaining = 0;
    fpData->end_enroll(fpData->tac_handle);
    return 0;
}

int fingerprint_idle(fingerprint_data_t *fpData)
{
    char property[PROPERTY_VALUE_MAX];
    int rc = 0;

    if(fpData->finger_press_state != FINGERPRINT_UP) {
        ALOGE("finger not up,error???");
    }
#ifdef DEBUG_FINGERPRINT
    if(property_get("debug.fingerprint.simulate", property, NULL) > 0) {
        if((!strncmp(property, "1", PROPERTY_VALUE_MAX )
            ||(!strncasecmp(property,"true", PROPERTY_VALUE_MAX )))) {
            fpData->simulate_enabled = true;
            ALOGE("enable simulate now");
        }else {
            fpData->simulate_enabled = false;
            ALOGE("disable simulate now");
        }
    }

    if(property_get("debug.fingerprint.saveimage", property, NULL) > 0) {
        if((!strncmp(property, "1", PROPERTY_VALUE_MAX )
            ||(!strncasecmp(property,"true", PROPERTY_VALUE_MAX )))) {
            fpData->saveimage_enabled = true;
            ALOGE("enable simulate now");
        }else {
            fpData->saveimage_enabled = false;
            ALOGE("disable simulate now");
        }
    }
#endif

    ALOGD( "FP_STATE: change from FINGERPRINT_UP --> FINGERPRINT_IDLE");
    fpData->finger_press_state = FINGERPRINT_IDLE;
	
	if(fpData->update_pending == true) {
		fpData->update_pending = false;
		fpData->store_template_db(fpData->tac_handle);
	}
	
    set_wakelock_state(fpData, false);
//	if(fpData->fb_blank != FB_BLANK_FP_BLANK) {
//		fpData->fb_blank = FB_BLANK_FP_BLANK;
//		fingerprint_sem_post(&fpData->display_sem);
//	}
    return 0;
}

int fingerprint_recovery(fingerprint_data_t *fpData)
{
    int ret = 0;
    char property[PROPERTY_VALUE_MAX];
    fpData->recovery_count++;
    if(fpData->recovery_count > RECOVERY_MAX_COUNT)
    {
    	ALOGE("try to recovery fingerprintTA too many times,do exit");
		fingerprint_cmd(fpData,fpData->evt_queue,FINGERPRINT_EVT_DISABLE_SENSOR);
		return -1;
    }	
		
    fpData->sensor_deinit(fpData);
    fpData->sensor_init(fpData);
    ret = fpData->sleep(fpData,!fpData->touch_mode);
    fpData->enroll_remaining = 0;
    fpData->finger_state = FINGERPRINT_STATE_LISTENING;
    fpData->finger_press_state = FINGERPRINT_IDLE;
    //ret = fpData->load_global_db(fpData->tac_handle, GLOBLE_DB_PATH, strlen(GLOBLE_DB_PATH) + 1);
    //if (ret != 0) {
    //    ALOGE("%s: fpsensor_load_global_db failed with error; %d, ignoring", __func__, ret);
    //    // Ignore load failure. The existing (empty) db will remain
    //}
    fingerprint_cmd(fpData,fpData->evt_queue,FINGERPRINT_EVT_INIT);
    return 0;
}

int fingerprint_do_detect(fingerprint_data_t *fpData)
{
    fingerprint_device_t *device = (fingerprint_device_t *)fpData;
    fingerprint_msg_t msg;
    finger_detect_type status;
    int detect_again=1;
    struct timeval tv;
    struct timeval ts_delta;
    int delta_us;
    int rc = 0;

    if(fpData->finger_press_state == FINGERPRINT_IDLE){
	    ALOGE("fingerprint enter idle,init sensor first");
	}

    status = (finger_detect_type)fpData->check_finger_present(fpData);
		
	ALOGE("fingerprint detect status %d",status);
    if(status == FINGER_DETECT_ERROR)
    {
	    ALOGE("fingerprint detect error %d,do recovery",status);
        fingerprint_recovery(fpData);
        return -1;
    }
	
    // check whether enrol timeout?
	if(fpData->finger_state == FINGERPRINT_STATE_ENROLLING){
        rc = gettimeofday(&tv, NULL);
        timersub(&fpData->enrol_timeout, &tv, &ts_delta);
        delta_us = TIME_IN_US(ts_delta);
        if(delta_us<=0) {
			msg.type = FINGERPRINT_ERROR;
			msg.data.error=FINGERPRINT_ERROR_TIMEOUT;
			fingerprint_end_enroll(fpData,&msg);
            ALOGD("enrol timeout!at detect");
        }
    }

    if (status == FINGER_DETECT_PRESENT) {
        if (fpData->finger_press_state == FINGERPRINT_WAIT_FINGERDOWN) {
            ALOGD( "FP_STATE: change from FINGERPRINT_WAIT_FINGERDOWN --> FINGERPRINT_DOWN");
            fpData->finger_press_state = FINGERPRINT_DOWN;
            detect_again = 0;

            if( fpData->state_notified == 0 && device->notify) {
                fpData->state_notified = 1;
				msg.type = FINGERPRINT_ACQUIRED;
				msg.data.acquired.acquired_info = FINGERPRINT_ACQUIRED_FINGERDOWN;
				device->notify(&msg);
			//	if(fpData->fb_blank != FB_BLANK_FP_UNBLANK) {
			//		fpData->fb_blank = FB_BLANK_FP_UNBLANK;
			//		fingerprint_sem_post(&fpData->display_sem);
			//	}
            }

            fingerprint_cmd(fpData,fpData->evt_queue,FINGERPRINT_EVT_CAPTURE);
        } else if (fpData->finger_press_state == FINGERPRINT_WAIT_FINGERUP) {
            usleep(3000);//detect too fast
        }
    }else if(status == FINGER_DETECT_LOST) {
        if( fpData->state_notified  && device->notify) {
             fpData->state_notified = 0;
			msg.type = FINGERPRINT_ACQUIRED;
			msg.data.acquired.acquired_info = FINGERPRINT_ACQUIRED_FINGERUP;
			device->notify(&msg);
        }
        detect_again = 0;
        ALOGD( "FP_STATE: change from %d --> FINGERPRINT_UP",fpData->finger_press_state);
        fpData->finger_press_state = FINGERPRINT_UP;
        rc = fpData->sleep(fpData,!fpData->touch_mode);
        rc = 0;
       // rc = ioctl(fpData->sysfs_fd,FPSENSOR_IOC_MASK_INTERRUPT,&rc);
        rc = gettimeofday(&fpData->idle_timeout, NULL);
		if(fpData->touch_mode && (fpData->selftest_pending == 0))
	        fpData->idle_timeout.tv_sec  += IDLE_TIMEOUT;
        fingerprint_sem_post(&fpData->poll_sem);
    }else if (fpData->finger_press_state == FINGERPRINT_WAIT_FINGERDOWN) {
        ;//ALOGD("finger still up,detect again");
    }

    if(detect_again == 1)   {
        fingerprint_cmd(fpData,fpData->evt_queue,FINGERPRINT_EVT_FINGER_DETECT);
        usleep(3000);//detect too fast
    }

    return 0;
}

int fingerprint_do_capture(fingerprint_data_t *fpData)
{
    fingerprint_device_t *device = (fingerprint_device_t *)fpData;
    fingerprint_msg_t msg;
    int ret = 0;
    if(fpData->finger_state == FINGERPRINT_STATE_LISTENING)
    {
        ALOGD( "FP_STATE: not in enroll or identify,ignore capture change from FINGERPRINT_DOWN --> FINGERPRINT_WAIT_FINGERUP");
        fpData->finger_press_state = FINGERPRINT_WAIT_FINGERUP;
        fingerprint_cmd(fpData,fpData->evt_queue,FINGERPRINT_EVT_FINGER_DETECT);
        return 0;
    }

    ret = fpData->capture_image(fpData);
    if (ret == FPSENSOR_ERROR_STATE) {
        ALOGD( "FP_STATE: capture fail change from FINGERPRINT_DOWN --> FINGERPRINT_WAIT_FINGERDOWN");
        fpData->finger_press_state = FINGERPRINT_WAIT_FINGERDOWN;
	 msg.type = FINGERPRINT_ACQUIRED;
        msg.data.acquired.acquired_info = FINGERPRINT_ACQUIRED_TOO_FAST;
        device->notify(&msg);	
        fingerprint_cmd(fpData,fpData->evt_queue,FINGERPRINT_EVT_FINGER_DETECT);
        return -EAGAIN;
    } else if (ret) {
        ALOGD( "FP_STATE: capture fail change from FINGERPRINT_DOWN --> FINGERPRINT_WAIT_FINGERDOWN");
        fpData->finger_press_state = FINGERPRINT_WAIT_FINGERDOWN;
        fingerprint_cmd(fpData,fpData->evt_queue,FINGERPRINT_EVT_FINGER_DETECT);
        return -EIO;
    }

#ifdef DEBUG_FINGERPRINT
    if(fpData->saveimage_enabled == true)
        ;//fpData->retrieve_image(fpData,"/data/fpimgs/fp.bmp");
#endif

    ALOGD( "FP_STATE: change from FINGERPRINT_DOWN --> FINGERPRINT_WAIT_FINGERUP");
    fpData->finger_press_state = FINGERPRINT_WAIT_FINGERUP;
    if(fpData->state_notified == 1) {
		fpData->state_notified = 2;
		msg.type = FINGERPRINT_ACQUIRED;
		msg.data.acquired.acquired_info = FINGERPRINT_ACQUIRED_WAIT_FINGERUP;
		device->notify(&msg);	
	}
    if(fpData->finger_state == FINGERPRINT_STATE_VERIFYING) {
        ALOGD( "goto verify");
        fingerprint_cmd(fpData,fpData->evt_queue,FINGERPRINT_EVT_IDENTIFY);
    }else if(fpData->finger_state == FINGERPRINT_STATE_ENROLLING) {
    ALOGD( "goto enroll");
#ifdef ENABLE_DUPLICATE_CHECK
        fingerprint_cmd(fpData,fpData->evt_queue,FINGERPRINT_EVT_DUPLICATE_CHECK);
#else
        fingerprint_cmd(fpData,fpData->evt_queue,FINGERPRINT_EVT_ENROL);
#endif
    }else
        fingerprint_cmd(fpData,fpData->evt_queue,FINGERPRINT_EVT_FINGER_DETECT);

    return 0;
}

int fingerprint_do_enroll(fingerprint_data_t *fpData)
{
    fingerprint_device_t *device = (fingerprint_device_t *)fpData;
    fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) fpData->tac_handle;
    fingerprint_msg_t msg;
    int ret = 0;
    int progress = 0;

    if(fpData->enroll_remaining == 0)
    {
        ALOGD("Enrol canceled,goto detect");
        return 0;
    }

    ret = fpData->enroll(fpData->tac_handle, NULL);

    if(ret == -FPSENSOR_ERROR_DUPLICATE){
		msg.type = FINGERPRINT_ACQUIRED;
        msg.data.acquired.acquired_info = FINGERPRINT_ACQUIRED_PARTIAL;
		device->notify(&msg);
        ALOGD("duplicate image,goto detect");
        return 0;
    }
    else if(ret) {
        ALOGD("enroll failed,goto detect");
        return 0;
    }

    fpData->enroll_remaining--;

    ALOGD("enroll_remaining is %d",fpData->enroll_remaining);
     if (fpData->enroll_remaining == 0) {

            ret = fpData->store_template_db(fpData->tac_handle);
            fpData->end_enroll(fpData->tac_handle);
            if (ret) {
				msg.type = FINGERPRINT_ERROR;
				msg.data.error=FINGERPRINT_ERROR_NO_SPACE;
				fingerprint_end_enroll(fpData,&msg);
                return -ENOMEM;
            }
            msg.data.enroll.finger.fid = tac_handle->enroll_fid + 1;
            msg.data.enroll.finger.gid = fpData->current_gid;
            msg.data.enroll.samples_remaining = 0;
            msg.type = FINGERPRINT_TEMPLATE_ENROLLING;
			fingerprint_end_enroll(fpData,&msg);
            ret = 0;
        } else {
            msg.data.enroll.finger.gid = fpData->current_gid;
            msg.data.enroll.finger.fid = 0;
            msg.data.enroll.samples_remaining = fpData->enroll_remaining;
            msg.type = FINGERPRINT_TEMPLATE_ENROLLING;
            device->notify(&msg);
        }

    return 0;
}


int fingerprint_begin_verify(fingerprint_device_t *device,uint64_t operation_id, uint32_t gid)
{
	fingerprint_data_t *fpData = (fingerprint_data_t *)device;
    fingerprint_msg_t msg;

    int status;

	if(fpData->image_mode == 0) {
        ALOGD("%s image mode not enable,can't verify\n", __func__);
        msg.data.error = FINGERPRINT_ERROR_NOT_ENABLED;
        status = -EPERM;
        goto out;
    }	

    if (gid != fpData->current_gid) {
        ALOGD("%s finger.gid != current_gid\n", __func__);
        msg.data.error = FINGERPRINT_ERROR_HW_UNAVAILABLE;
        status = -EINTR;
		goto out;
    }	

	fpData->finger_state = FINGERPRINT_STATE_VERIFYING;
    if(fpData->state_notified == 0) {
		msg.type = FINGERPRINT_ACQUIRED;
		msg.data.acquired.acquired_info = FINGERPRINT_ACQUIRED_WAIT_FINGERDOWN;
		device->notify(&msg);	
	}
    if(fpData->finger_press_state == FINGERPRINT_WAIT_FINGERUP)
        fpData->finger_press_state = FINGERPRINT_WAIT_FINGERDOWN;
	fpData->sleep(fpData,!fpData->touch_mode);
	return 0;
out:
	ALOGE("begin verify error");
	msg.type = FINGERPRINT_ERROR;
	fingerprint_end_verify(fpData,&msg);
	return status;
}

int fingerprint_do_verify(fingerprint_device_t *device)
{
#if 1
	fingerprint_data_t *fpData = (fingerprint_data_t *)device;
    fingerprint_msg_t msg;
    fpsensor_identify_result_t identify_data;
    int ret = 0;

    if(fpData->finger_state != FINGERPRINT_STATE_VERIFYING) {
        ALOGD( "Not in identify state,don't do identify");
        fingerprint_cmd(fpData,fpData->evt_queue,FINGERPRINT_EVT_FINGER_DETECT);
        return 0;
    }

    msg.type = FINGERPRINT_ACQUIRED;
    msg.data.acquired.acquired_info = FINGERPRINT_ACQUIRED_GOOD;
    device->notify(&msg);


    ret = fpData->verify(fpData->tac_handle, NULL, &identify_data);
    if (ret) {
        ret = -EIO;
        goto out;
    }

    msg.type = FINGERPRINT_AUTHENTICATED;

    if (identify_data.result >= 1) {
        msg.data.authenticated.finger.fid = identify_data.fid;
        msg.data.authenticated.finger.gid = fpData->current_gid;
		fingerprint_end_verify(fpData,&msg);

        if (identify_data.result == 2) {
            fpData->update_pending = true;
        }

    } else {
	    msg.data.authenticated.finger.gid = fpData->current_gid;
	    msg.data.authenticated.finger.fid = 0;
		device->notify(&msg);
    }
out:
    if (ret) {
        ALOGE("%s failed %i\n", __func__, ret);
        msg.type = FINGERPRINT_ERROR;
        switch (ret) {
        case -EINTR:
            break;
        default:
            msg.data.error = FINGERPRINT_ERROR_HW_UNAVAILABLE;
            device->notify(&msg);
            break;
        }
    }
    fingerprint_cmd(fpData,fpData->evt_queue,FINGERPRINT_EVT_FINGER_DETECT);
#endif
    return 0;
}

int fingerprint_end_verify(fingerprint_data_t *fpData,fingerprint_msg_t *msg)
{
    fingerprint_device_t *device = (fingerprint_device_t *)fpData;
    fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*)fpData->tac_handle;
    if(msg)
        device->notify(msg);
    fpData->finger_state = FINGERPRINT_STATE_LISTENING;
   // fpData->end_verify(fpData->tac_handle);
    return 0;
}


int fingerprint_handle_poll_result(fingerprint_data_t *fpData,uint32_t timeout)
{
    int rc = 0;
    int mask = 0;
    struct timeval tv;
    struct timeval ts_delta;
    int delta_us = 0;

    //poll irq succes
    if(timeout == 0) {
        mask = 1;
        //rc = ioctl(fpData->sysfs_fd,FPSENSOR_IOC_MASK_INTERRUPT,&mask);
        if(fpData->finger_press_state == FINGERPRINT_IDLE) {
			ALOGD("poll result:do sensor init");
            //vibrator_on(1000);
            fingerprint_cmd(fpData,fpData->evt_queue,FINGERPRINT_EVT_INIT);
        }else {
            ALOGD( "FP_STATE: poll_result:change from FINGERPRINT_UP --> FINGERPRINT_WAIT_FINGERDOWN");
            fpData->finger_press_state = FINGERPRINT_WAIT_FINGERDOWN;
            fingerprint_cmd(fpData,fpData->evt_queue,FINGERPRINT_EVT_FINGER_DETECT);
        }
    }else if(timeout > 0) {//timeout
        gettimeofday(&tv, NULL);
        //for enrol timeout
        if(fpData->finger_state == FINGERPRINT_STATE_ENROLLING){
            timersub(&fpData->enrol_timeout, &tv, &ts_delta);
            delta_us = TIME_IN_US(ts_delta);
            if(delta_us<=0) {
				fingerprint_msg_t msg;
				msg.type = FINGERPRINT_ERROR;
				msg.data.error=FINGERPRINT_ERROR_TIMEOUT;
				fingerprint_end_enroll(fpData,&msg);
                ALOGD("enrol timeout!at poll");
            }
        }

        //for autosuspend
        if(fpData->finger_press_state != FINGERPRINT_IDLE) {
            timersub(&fpData->idle_timeout, &tv, &ts_delta);
            delta_us = TIME_IN_US(ts_delta);
            if(delta_us<=0) {
                ALOGD("poll timeout!enter idle.");
                fingerprint_idle(fpData);
            }
        }

        //poll timeout,repoll it
        fingerprint_sem_post(&fpData->poll_sem);
    }

    return 0;
}
/*************************************************************************
pollthread:
poll irq,use as timer to implement enrol timeout and idle timeout
*************************************************************************/
static void* poll_thread(void *data)
{
    fingerprint_data_t *fpData = (fingerprint_data_t *)data;
    struct pollfd pfd;
    int rc = 0;
    struct timeval tv;
    struct timeval ts_delta;
    int delta_us = 0;
    int timeout= 10000;
    long int random = 0;
    int32_t poll_val;
    fingerprint_cmd_t *node = NULL;

    pfd.fd = fpData->sysfs_fd;
    pfd.events = POLLIN | POLLHUP | POLLERR;
    while(1) {
        do {
               rc = fingerprint_sem_wait(&fpData->poll_sem);
               if (rc != 0 && errno != EINVAL) {
                   ALOGE("%s: fingerprint_sem_wait error (%s)",
                              __func__, strerror(errno));
                   break;
               }
        } while (rc != 0);

        timeout= 10000;

        gettimeofday(&tv, NULL);
        if(fpData->finger_state == FINGERPRINT_STATE_ENROLLING){
            timersub(&fpData->enrol_timeout, &tv, &ts_delta);
            delta_us = TIME_IN_US(ts_delta);
            if(delta_us>0) {
                timeout = delta_us/1000;
                ALOGD("wait %d ms for enrol timeout",timeout);
            } else 
				timeout = 1;
        }else if(fpData->finger_press_state != FINGERPRINT_IDLE) {
            timersub(&fpData->idle_timeout, &tv, &ts_delta);
            delta_us = TIME_IN_US(ts_delta);
            if(delta_us > 0) {
                timeout = delta_us/1000;
                ALOGD("wait %d ms for idle timeout",timeout);
			} else 
				timeout = 1;
        }

        poll_val = -1;
        rc = poll(&pfd, 1, timeout);
        if (!rc) {
            poll_val=timeout;
        } else if (rc < 0) {
            ALOGE("Error while polling: %d\n", rc);
        } else if (pfd.revents & POLLIN) {
            poll_val = 0;
        } else if (pfd.revents & POLLERR) {
            ALOGE("POLL Error; error no = %d", errno);
        } else if (pfd.revents & POLLHUP) {
            ALOGE("Remote side hung up");
        } else {
            ALOGD("Unkown events = 0x%x\n", pfd.revents);
        }
		
#ifdef DEBUG_FINGERPRINT
        if(fpData->simulate_enabled == true) {
            random = rand();
            rc = random%10;
            if(rc >= 2){
                ALOGD( "simulate irq happened");
                poll_val = 0;
            }
        }
#endif		

        if(poll_val >= 0) {
            node = (fingerprint_cmd_t *)malloc(sizeof(fingerprint_cmd_t));
            if (NULL == node) {
                ALOGE("%s: No memory for fingerprint_cmd_t", __func__);
                continue;
            }

            memset(node, 0, sizeof(fingerprint_cmd_t));
            node->cmdId= FINGERPRINT_CMD_POLLIN;
            node->data = poll_val;
            if (fpData->cmd_queue->enqueue((void *)node) == true)
                fingerprint_sem_post(&fpData->cmd_sem);
        }else {
            ALOGD( "poll error,do poll again");
            fingerprint_sem_post(&fpData->poll_sem);
        }

        if(fpData->stop_thread == true)
            break;
    }
    ALOGD("EXIT poll()\n");
    return NULL;
}

static void* cmp1_thread(void *data)
{
    fingerprint_data_t *fpData = (fingerprint_data_t *)data;
    int rc = 0;
    struct timeval tv;
    struct timeval ts_delta;
    int delta_us = 0;
    int cmp_val = 0;
    fingerprint_cmd_t *node = NULL;

    while(1) {
        do {
               rc = fingerprint_sem_wait(&fpData->cmp1_sem);
               if (rc != 0 && errno != EINVAL) {
                   ALOGE("%s: fingerprint_sem_wait error (%s)",
                              __func__, strerror(errno));
                   break;
               }
        } while (rc != 0);

        gettimeofday(&tv, NULL);
        if(cmp_val >= 0) {
            node = (fingerprint_cmd_t *)malloc(sizeof(fingerprint_cmd_t));
            if (NULL == node) {
                ALOGE("%s: No memory for fingerprint_cmd_t", __func__);
                continue;
            }

            memset(node, 0, sizeof(fingerprint_cmd_t));
            node->cmdId= FINGERPRINT_CMD_CMP1;
            node->data = cmp_val;
            if (fpData->cmd_queue->enqueue((void *)node) == true)
                fingerprint_sem_post(&fpData->cmp1_sem);
        }else {
            ALOGD( "cmp error,do cmp again");
            fingerprint_sem_post(&fpData->cmp1_sem);
        }

        if(fpData->stop_thread == true)
            break;
    }
    ALOGD("EXIT cmp()\n");
    return NULL;
}

static void* cmp2_thread(void *data)
{
    fingerprint_data_t *fpData = (fingerprint_data_t *)data;
    int rc = 0;
    struct timeval tv;
    struct timeval ts_delta;
    int delta_us = 0;
    int cmp_val = 0;
    fingerprint_cmd_t *node = NULL;

    while(1) {
        do {
               rc = fingerprint_sem_wait(&fpData->cmp2_sem);
               if (rc != 0 && errno != EINVAL) {
                   ALOGE("%s: fingerprint_sem_wait error (%s)",
                              __func__, strerror(errno));
                   break;
               }
        } while (rc != 0);

        gettimeofday(&tv, NULL);
        if(cmp_val >= 0) {
            node = (fingerprint_cmd_t *)malloc(sizeof(fingerprint_cmd_t));
            if (NULL == node) {
                ALOGE("%s: No memory for fingerprint_cmd_t", __func__);
                continue;
            }

            memset(node, 0, sizeof(fingerprint_cmd_t));
            node->cmdId= FINGERPRINT_CMD_CMP2;
            node->data = cmp_val;
            if (fpData->cmd_queue->enqueue((void *)node) == true)
                fingerprint_sem_post(&fpData->cmp2_sem);
        }else {
            ALOGD( "cmp error,do cmp again");
            fingerprint_sem_post(&fpData->cmp2_sem);
        }

        if(fpData->stop_thread == true)
            break;
    }
    ALOGD("EXIT cmp()\n");
    return NULL;
}

static void* cmp3_thread(void *data)
{
    fingerprint_data_t *fpData = (fingerprint_data_t *)data;
    int rc = 0;
    struct timeval tv;
    struct timeval ts_delta;
    int delta_us = 0;
    int cmp_val = 0;
    fingerprint_cmd_t *node = NULL;

    while(1) {
        do {
               rc = fingerprint_sem_wait(&fpData->cmp3_sem);
               if (rc != 0 && errno != EINVAL) {
                   ALOGE("%s: fingerprint_sem_wait error (%s)",
                              __func__, strerror(errno));
                   break;
               }
        } while (rc != 0);

        gettimeofday(&tv, NULL);
        if(cmp_val >= 0) {
            node = (fingerprint_cmd_t *)malloc(sizeof(fingerprint_cmd_t));
            if (NULL == node) {
                ALOGE("%s: No memory for fingerprint_cmd_t", __func__);
                continue;
            }

            memset(node, 0, sizeof(fingerprint_cmd_t));
            node->cmdId= FINGERPRINT_CMD_CMP3;
            node->data = cmp_val;
            if (fpData->cmd_queue->enqueue((void *)node) == true)
                fingerprint_sem_post(&fpData->cmp3_sem);
        }else {
            ALOGD( "cmp error,do cmp again");
            fingerprint_sem_post(&fpData->cmp3_sem);
        }

        if(fpData->stop_thread == true)
            break;
    }
    ALOGD("EXIT cmp()\n");
    return NULL;
}


/*************************************************************************
cmdthread:
handle cmd,end do state convert
*************************************************************************/
void *cmd_thread(void *data)
{
    int ret = 0;
    fingerprint_data_t *fpData = (fingerprint_data_t *)data;
    fingerprint_device_t *device = (fingerprint_device_t *)data;
    fingerprint_msg_t msg;
	bool reserved_node;
    do {
        do {
            ret = fingerprint_sem_wait(&fpData->cmd_sem);
            if (ret != 0 && errno != EINVAL) {
                ALOGE("%s: fingprint_sem_wait error (%s)",
                           __func__, strerror(errno));
                return NULL;
            }
        } while (ret != 0);

        // we got notified about new cmd avail in cmd queue
        // first check API cmd queue
        fingerprint_cmd_t *node = (fingerprint_cmd_t *)fpData->cmd_queue->dequeue();
        if (node == NULL) {
            // no API cmd, then check evt cmd queue
            node = (fingerprint_cmd_t *)fpData->evt_queue->dequeue();
        }

        if (node == NULL) {
            //ALOGE("%s: no cmd,continue",__func__);
            continue;
        }
		reserved_node = node->reserved_node;
        switch (node->cmdId) {
        case FINGERPRINT_CMD_SET_MODE:
		if(fpData->disable_sensor == true) {
		    ALOGD("fingerprint sensor error,can't setmode");
		    break;
		}			
		if((uint32_t)node->data ==	IMAGE_MODE) {
			if(node->len  ==(uint32_t)fpData->image_mode) {
				ALOGD("fingerprint already %s",fpData->image_mode == 1?"enabled":"disabled");
				break;
			}
			fpData->image_mode = node->len;
			ALOGD("fingerprint set image_mode %s",fpData->touch_mode == 1?"enabled":"disabled");
			break;
		}
		else if((uint32_t)node->data ==	TOUCH_MODE) {
			if(node->len  ==(uint32_t)fpData->touch_mode) {
				ALOGD("fingerprint already %s",fpData->touch_mode == 1?"enabled":"disabled");
				break;
			}
			fpData->touch_mode = node->len;
			ALOGD("fingerprint set touch_mode %s",fpData->touch_mode == 1?"enabled":"disabled");
		}
		else
		{
			ALOGD("Unkown item.");
			break;
		}

		if(fpData->finger_press_state != FINGERPRINT_IDLE) {
		    ALOGD("do sleep in state machine");
		    break;
		}
	//		ret = fpData->sleep(fpData,!fpData->touch_mode);
	//		if(fpData->finger_state == FINGERPRINT_STATE_IDLE) {
	//			fpData->finger_state = FINGERPRINT_STATE_LISTENING;
	//			fingerprint_cmd(fpData,fpData->evt_queue,FINGERPRINT_EVT_INIT);
	//		}else if(ioctl(fpData->sysfs_fd,FPSENSOR_IOC_WAKEUP_POLL,NULL))
	//		{
	//			ALOGE("Failed to wakeup poll thread in %s func with error %d: %s\n",__func__,errno,strerror(errno));
	//		}
	//	}else if(fpData->touch_mode == 0){
	//		ret = 1;
	//		ret = ioctl(fpData->sysfs_fd,FPSENSOR_IOC_MASK_INTERRUPT,&ret);
	//		ret = fpData->sleep(fpData,!fpData->touch_mode);
	//		ret = 0;
	//		ret = ioctl(fpData->sysfs_fd,FPSENSOR_IOC_MASK_INTERRUPT,&ret);
	//	}
            break;
			
	 case FINGERPRINT_CMD_PRE_ENROL:			
            ALOGD("cmd_thread:handle FINGERPRINT_CMD_PRE_ENROL");
		    // ret = fpData->get_hw_auth_challenge(fpData->tac_handle, &node->data);	
			//if (ret) {
		    //    ALOGE("%s failed %i\n", __func__, ret);
		    //    node->data = 0;
		    //}
            node->data = get_64bit_rand();
		    ALOGD("FINGERPRINT_CMD_PRE_ENROL challenge %llu\n", node->data);
			fingerprint_sem_post(&fpData->sync_sem);
		break;
		
	 case FINGERPRINT_CMD_POST_ENROL:	
            ALOGD("cmd_thread:handle FINGERPRINT_CMD_POST_ENROL");

		    //ret = fpData->get_hw_auth_challenge(fpData->tac_handle, &node->data);	
		    //if (ret) {
		    //    ALOGE("%s failed %i\n", __func__, ret);
		    //    node->data = -1;
		    //}
            node->data = get_64bit_rand();
		    ALOGD("FINGERPRINT_CMD_POST_ENROL challenge %llu",node->data);
			fingerprint_sem_post(&fpData->sync_sem);
		break;	
		
	case FINGERPRINT_CMD_GET_AUTH_ID:
		ALOGD("cmd_thread:handle FINGERPRINT_CMD_GET_AUTH_ID");
		    //ret = fpData->get_template_db_id(fpData->tac_handle, &node->data);
		    //if (ret) {
		    //    ALOGE("FINGERPRINT_CMD_GET_AUTH_ID failed %i\n", ret);
		    //    node->data = 0;
		    //}
            node->data =  ((fpsensor_handle_internal_t*)fpData->tac_handle)->auth_id;
		    ALOGD("FINGERPRINT_CMD_GET_AUTH_ID id %d", node->data);
			fingerprint_sem_post(&fpData->sync_sem);
		break;
		
	case FINGERPRINT_CMD_SET_ACTIVE_GRP:{
			ALOGD("cmd_thread:handle FINGERPRINT_CMD_SET_ACTIVE_GRP");
		    char filename[PATH_MAX];

		    int length = snprintf(filename, PATH_MAX, "%s/",node->buf);

		    ret = fpData->load_user_db(fpData, filename, length + 1);
		    if (ret != 0) {
		        ALOGE("%s: load_user_db failed with error; %d", __func__, ret);
		        ret = 0; // Ignore load failure. The existing (empty) db will remain
		    }

		   // ret = fpData->set_active_fingerprint_set(fpData->tac_handle, (int32_t)node->data);
		   // if (ret) {
		   //     ALOGE("%s failed %i\n", __func__, ret);
		   //     node->data = -1;		    
		   //  }
           // else 
             {
		   	 fpData->current_gid = node->data;
			 node->data = 0;
		     }
			fingerprint_sem_post(&fpData->sync_sem);
		}break;
		
    case  FINGERPRINT_CMD_CALIBRATE:{
			ALOGD("cmd_thread:handle FINGERPRINT_CMD_CALIBRATE");
		    ret = fpData->calibrate(fpData, node->buf);
		    if (ret != 0) {
		        ALOGE("%s: calibrate fail with error %d", __func__, ret);
		    }
            node->status = ret;
			fingerprint_sem_post(&fpData->sync_sem);
		}break;
    
    case FINGERPRINT_CMD_ENUMERATE:
			ALOGD("cmd_thread:handle FINGERPRINT_CMD_ENUMERATE");
			node->status = fingerprint_do_enumerate(device,(fingerprint_finger_id_t *)node->buf,&node->len);
			fingerprint_sem_post(&fpData->sync_sem);
		break;

    case FINGERPRINT_CMD_CANCEL:
            ALOGD("cmd_thread:handle FINGERPRINT_CMD_CANCEL");
	     if(fpData->finger_state == FINGERPRINT_STATE_ENROLLING)
	     {
				fingerprint_msg_t msg;
				msg.type = FINGERPRINT_ERROR;
				msg.data.error=FINGERPRINT_ERROR_CANCELED;
				fingerprint_end_enroll(fpData,&msg);
	     }else  if(fpData->finger_state == FINGERPRINT_STATE_VERIFYING) {
	            ALOGD("cmd_thread:handle FINGERPRINT_CMD_CANCELVERIFY");
	            fpData->finger_state = FINGERPRINT_STATE_LISTENING;
				//(void) fpData->end_verify(fpData->tac_handle);
		}
            break;

        case FINGERPRINT_CMD_REMOVE:
            ALOGD("cmd_thread:handle FINGERPRINT_CMD_REMOVE");
		fingerprint_do_remove(device,node->len, (uint32_t)node->data);			
		//	if(fpData->update_pending && fpData->finger_press_state == FINGERPRINT_IDLE) {
			//	fpData->update_pending = false;
				//fpData->update_templates(fpData);
			//}
           break;


        case FINGERPRINT_CMD_ENROLL:
            ALOGD("cmd_thread:handle FINGERPRINT_CMD_ENROLL");
            fingerprint_enroll_init(fpData,(const hw_auth_token_t *)node->buf,node->len,(uint32_t)node->data);
            break;

        case FINGERPRINT_CMD_VERIFY:
			ALOGD("cmd_thread:handle FINGERPRINT_CMD_VERIFY");
			fingerprint_begin_verify(device,node->data,node->len);
            break;

        case FINGERPRINT_CMD_POLLIN:
            fingerprint_handle_poll_result(fpData,node->data);
            break;

        case FINGERPRINT_EVT_INIT:
            //ALOGD("cmd_thread:handle FINGERPRINT_EVT_INIT");
            set_wakelock_state(fpData, true);
            ALOGD( "FP_STATE: change from FINGERPRINT_IDLE --> FINGERPRINT_WAIT_FINGERDOWN");
            fpData->finger_press_state = FINGERPRINT_WAIT_FINGERDOWN;
            fingerprint_cmd(fpData,fpData->evt_queue,FINGERPRINT_EVT_FINGER_DETECT);
            break;

        case FINGERPRINT_EVT_FINGER_DETECT:
            //ALOGE("cmd_thread:handle FINGERPRINT_EVT_FINGER_DETECT");
            ret = fingerprint_do_detect(fpData);
            break;

        case FINGERPRINT_EVT_CAPTURE:
            //ALOGE("cmd_thread:handle FINGERPRINT_EVT_CAPTURE");
            fingerprint_do_capture(fpData);
            break;

        case FINGERPRINT_EVT_DUPLICATE_CHECK:
            //ALOGE("cmd_thread:handle FINGERPRINT_EVT_DUPLICATE_CHECK");
            break;

        case FINGERPRINT_EVT_ENROL:
            //ALOGE("cmd_thread:handle FINGERPRINT_EVT_ENROL");
            fingerprint_do_enroll(fpData);
            fingerprint_cmd(fpData,fpData->evt_queue,FINGERPRINT_EVT_FINGER_DETECT);
            break;

        case FINGERPRINT_EVT_IDENTIFY:
            ALOGE("cmd_thread:handle FINGERPRINT_EVT_IDENTIFY");
            fingerprint_do_verify(device);
            break;

	 case FINGERPRINT_EVT_DISABLE_SENSOR:
	 	fpData->evt_queue->flush();
		fpData->disable_sensor = true;
		fpData->touch_mode = 0;
		fpData->image_mode = 0;
		break;
		
        case FINGERPRINT_CMD_EXIT:
            ALOGE("cmd_thread:handle FINGERPRINT_CMD_EXIT");
            fingerprint_idle(fpData);
			fpData->sensor_deinit(fpData);
            fpData->stop_thread = true;
            fingerprint_sem_post(&fpData->poll_sem);
            set_wakelock_state(fpData, false);
            if(device->notify) {
                msg.type = FINGERPRINT_EXIT;
                device->notify(&msg);
                device->notify = NULL;
            }
            break;
			
        default:
            ALOGE("Unkown cmd");
        }
        //ALOGE("handle cmdId %d finished",node->cmdId);
        if(reserved_node == false)
        {
	        free(node);
	        node = NULL;
        }
    } while (fpData->stop_thread == false);
    ALOGE("fingerprint cmd thread exit running");
    return NULL;
}


static int fingerprint_open(const hw_module_t* module, const char __unused *id,
                            hw_device_t** device)
{
    int ret = 0;
    char property[PROPERTY_VALUE_MAX];
    unsigned int i=0;
    int fpid=0;

    if (device == NULL) {
        ALOGE("NULL device on open");
        return -EINVAL;
    }

    fingerprint_data_t *fpData = (fingerprint_data_t *)malloc(sizeof(fingerprint_data_t));
    fingerprint_device_t *dev= (fingerprint_device_t *)fpData;
    memset(fpData, 0, sizeof(fingerprint_data_t));

    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = HARDWARE_MODULE_API_VERSION(1, 0);
    dev->common.module = (struct hw_module_t*) module;
    dev->common.close = fingerprint_close;

    dev->pre_enroll = fingerprint_pre_enroll;
    dev->enroll = fingerprint_enroll;
    dev->post_enroll = fingerprint_post_enroll;	
    dev->get_authenticator_id = fingerprint_get_auth_id;
    dev->set_active_group = fingerprint_set_active_group;
    dev->calibrate = fingerprint_calibrate;
    dev->authenticate = fingerprint_authenticate;
    dev->cancel = fingerprint_cancel;
    dev->enumerate = fingerprint_enumerate;
    dev->remove = fingerprint_remove;
    dev->set_notify = set_notify_callback;
    dev->notify = NULL;
	
	fpData->sysfs_fd = open(FINGERPRINT_SENSOR_PATH, O_RDWR);
		if(fpData->sysfs_fd<0)
		{
			ALOGE("Failed to open %s with error %d: %s\n",FINGERPRINT_SENSOR_PATH,errno,strerror(errno));
			ret	= property_set("persist.yulong.fpModuleName", "No Sensor");
			return -ENODEV;
		}
		
	//	rc = ioctl(device->sysfs_fd,FPSENSOR_IOC_GET_MODULE_NAME,property);
	//	if(rc < 0)
	//	{
	//		ALOGE("Failed to read fingerprint module's name in %s func with error %d: %s\n",__func__,errno,strerror(errno));
	//	}
		

    dfs747_device_init(fpData);

    ret = fpData->sensor_init(fpData);
    if(ret)
    {
        ALOGE("fingerprint_sensor_init fail,check fpc lib or memory");
     	 ret = -1;
        goto err_probe_sensor;
    }	

				
    fpData->cmd_queue = new fingerprintQueue();
    fpData->evt_queue = new fingerprintQueue();
    fingerprint_sem_init(&fpData->cmd_sem, 0);
    fingerprint_sem_init(&fpData->poll_sem, 0);
    //fpData->algorithm = new fingerprint_algorithm();
    fpData->state_notified = 0;
    fpData->kpi_enabled = true;
    fpData->finger_press_state = FINGERPRINT_IDLE;
    fpData->finger_state = FINGERPRINT_STATE_IDLE;
	//fpData->finger_state = FINGERPRINT_STATE_VERIFYING;
    *device = (hw_device_t*) dev;
    fpData->stop_thread = false;

    ret = pthread_create(&fpData->cmd_pid,
                   NULL,
                   cmd_thread,
                   fpData);
    if (ret) {
        ALOGE("Failed to create cmd thread: %d\n", ret);
    }
    pthread_setname_np(fpData->cmd_pid, "fingerprint cmd thread");

    ret = pthread_create(&fpData->poll_pid,
                   NULL,
                   poll_thread,
                   fpData);
    if (ret) {
         ALOGE("Failed to create poll thread: %d\n", ret);
    }
    pthread_setname_np(fpData->poll_pid, "fingerprint poll thread");

    ret = pthread_create(&fpData->cmp1_pid,
                   NULL,
                   cmp1_thread,
                   fpData);
    if (ret) {
         ALOGE("Failed to create cmp1 thread: %d\n", ret);
    }
    pthread_setname_np(fpData->cmp2_pid, "fingerprint cmp1 thread");

    ret = pthread_create(&fpData->cmp2_pid,
                   NULL,
                   cmp2_thread,
                   fpData);
    if (ret) {
         ALOGE("Failed to create cmp2 thread: %d\n", ret);
    }
    pthread_setname_np(fpData->cmp2_pid, "fingerprint cmp2 thread");

    ret = pthread_create(&fpData->cmp3_pid,
                   NULL,
                   cmp3_thread,
                   fpData);
    if (ret) {
         ALOGE("Failed to create cmp3 thread: %d\n", ret);
    }
    pthread_setname_np(fpData->cmp3_pid, "fingerprint cmp3 thread");
//    fpData->touch_mode = 1;
	fpData->image_mode = 1;

	//fpData->finger_state = FINGERPRINT_STATE_LISTENING;
   // fingerprint_cmd_wait(fpData,FINGERPRINT_CMD_SET_ACTIVE_GRP,(uint64_t)0,NULL);
	fingerprint_cmd(fpData,fpData->evt_queue,FINGERPRINT_EVT_INIT);

    return 0;
   
err_probe_sensor:
    ALOGE("err_probe_sensor: %d\n", ret);
	free(fpData->tac_handle);
err_tac_open:
    ALOGE("err_tac_open: %d\n", ret);
    if(fpData->sysfs_fd)
     close(fpData->sysfs_fd);
err_device:
    ALOGE("err_device: %d\n", ret);
    if(fpData)
     free(fpData);
    return ret;
}

static struct hw_module_methods_t fingerprint_module_methods = {
    .open = fingerprint_open,
};

fingerprint_module_t HAL_MODULE_INFO_SYM = {
    .common = {
        .tag                = HARDWARE_MODULE_TAG,
        .module_api_version = FINGERPRINT_MODULE_API_VERSION_2_0,
        .hal_api_version    = HARDWARE_HAL_API_VERSION,
        .id                 = FINGERPRINT_HARDWARE_MODULE_ID,
        .name               = "Fingerprint HAL",
        .author             = "The Android Open Source Project",
        .methods            = &fingerprint_module_methods,
        .dso                = NULL,
        .reserved           = {0},
    },
};
