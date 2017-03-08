#include <stdio.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <utils/Log.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <hardware/hardware.h>
#include "fingerprint_queue.h"
#include "fingerprint_semaphore.h"


#define MAX_TEMPLATE_COUNT 7  /*max fingerprint templates support*/


#define TIME_IN_US(t) (t.tv_sec * 1000000LL + t.tv_usec)

#define IMAGE_MODE      1
#define TOUCH_MODE		2

enum  fingerprint_enroll_result {
     ENROL_SUCCESSFUL = 0,
     ENROL_BADIMAGE = -1,
     ENROL_FEATURELOW = -2,
     ENROL_FAILED_EXCEPTION = -3,
};

typedef enum fingerprint_press_state {
    FINGERPRINT_DOWN = 1,
    FINGERPRINT_UP = 2,
    FINGERPRINT_WAIT_FINGERDOWN = 3,
    FINGERPRINT_WAIT_FINGERUP = 4,
    FINGERPRINT_IDLE = 5,
} fingerprint_press_state_t;

typedef enum fingerprint_state {
    FINGERPRINT_STATE_IDLE = 0,
    FINGERPRINT_STATE_LISTENING = 1,
    FINGERPRINT_STATE_ENROLLING = 2,
    FINGERPRINT_STATE_REMOVING = 3,
    FINGERPRINT_STATE_VERIFYING = 4,
} fingerprint_state_t;
  

/* Security levels for the matching function */
typedef enum {
    /* FAR 1/1000 */
    SECURITY_LOW,
    /* FAR 1/10,000 */
    SECURITY_REGULAR,
    /* FAR 1/50,000 */
    SECURITY_HIGH,
    /* FAR 1/100,000 */
    SECURITY_VERY_HIGH,
} fingerprint_security_t;

typedef enum
{
    FINGERPRINT_CMD_SET_USERID,
    FINGERPRINT_CMD_PRE_ENROL,
    FINGERPRINT_CMD_POST_ENROL,	
    FINGERPRINT_CMD_GET_AUTH_ID,	
    FINGERPRINT_CMD_SET_ACTIVE_GRP,	
    FINGERPRINT_CMD_ENROLL,
    FINGERPRINT_CMD_CANCEL,
    FINGERPRINT_CMD_REMOVE,
    FINGERPRINT_CMD_ENUMERATE,
    FINGERPRINT_CMD_GET_NAME,
    FINGERPRINT_CMD_SET_NAME,
    FINGERPRINT_CMD_VERIFY,
    FINGERPRINT_CMD_SET_MODE,
    FINGERPRINT_CMD_EXIT,
    FINGERPRINT_CMD_POLLIN,
    FINGERPRINT_CMD_CMP1,
    FINGERPRINT_CMD_CMP2,
    FINGERPRINT_CMD_CMP3,
    FINGERPRINT_CMD_CALIBRATE,
    FINGERPRINT_CMD_SELFTEST,
    FINGERPRINT_CMD_API_MAX,

    FINGERPRINT_EVT_INIT=100,
    FINGERPRINT_EVT_FINGER_DETECT,
    FINGERPRINT_EVT_CAPTURE,
    FINGERPRINT_EVT_ENROL,
    FINGERPRINT_EVT_IDENTIFY,
    FINGERPRINT_EVT_DUPLICATE_CHECK,
    FINGERPRINT_EVT_DISABLE_SENSOR,
    FINGERPRINT_EVT_MAX=255
} fingerprint_cmdID_t;

typedef struct {
     fingerprint_cmdID_t cmdId;
     uint64_t data;
     uint32_t len;
     char	 * buf; 
     bool     reserved_node;
     int32_t status;
} fingerprint_cmd_t;


class fingerprintQueue;
typedef struct fingerprint_data {
	fingerprint_device_t device; //inheritance
	int32_t (*sensor_init)(fingerprint_data* device);
	int32_t (*sensor_deinit)(fingerprint_data* device);
	int32_t (*sleep)(fingerprint_data* device,int sleep);
	int32_t (*check_finger_present)(fingerprint_data* device);                                
	int32_t (*capture_image)(fingerprint_data* device);
	int32_t (*calibrate)(fingerprint_data* device,const char *store_path);
	int32_t (*begin_enroll)(void *pHandle);
	int32_t (*enroll)(void *pHandle, void *enroll_data);
	int32_t (*end_enroll)(void *pHandle);

	int32_t (*begin_verify)(void *pHandle,uint32_t *indices, uint32_t index_count);
	int32_t (*verify)(void *pHandle, const uint8_t* nonce, void* identify_data);
	int32_t (*end_verify)(void *pHandle);       

	int32_t (*set_hw_auth_challenge)(void *pHandle,uint64_t challenge);                       
	int32_t (*get_hw_auth_challenge)(void *pHandle, uint64_t* challenge);                       
	int32_t (*validate_auth_challenge)(void *pHandle, const uint8_t* auth_token, uint32_t size_token);
	int32_t (*get_hw_auth_token)(void *pHandle,uint8_t* auth_token, uint32_t size_auth_token);
	int32_t (*load_user_db)(fingerprint_data* device, const char* path, uint32_t path_len);                          
	int32_t (*store_template_db)(void *pHandle);                               
	int32_t (*delete_template)(void *pHandle, uint32_t index);                   
	int32_t (*set_active_fingerprint_set)(void *pHandle, int32_t fingerprint_set_key);      

 
    fingerprintQueue *cmd_queue; /*  cmd queue for APIs */
    fingerprintQueue *evt_queue;  /* cmd queue for evt from statemachine */
    pthread_t cmd_pid;           /* cmd thread ID */
    pthread_t poll_pid;           /* poll thread ID */
    pthread_t cmp1_pid;           /* cmp tmpl thread ID 1*/
    pthread_t cmp2_pid;           /* cmp tmpl thread ID 2*/
    pthread_t cmp3_pid;           /* cmp tmpl thread ID 3*/

    bool stop_thread;
    fingerprint_semaphore_t cmd_sem;               /* semaphore for cmd thread */
    fingerprint_semaphore_t sync_sem;               /* semaphore for cmd thread */
    fingerprint_semaphore_t poll_sem;              /* semaphore for synchronized with poll thread */
    fingerprint_semaphore_t cmp1_sem;              /* semaphore for synchronized with cmd thread */
    fingerprint_semaphore_t cmp2_sem;              /* semaphore for synchronized with cmd thread */
    fingerprint_semaphore_t cmp3_sem;              /* semaphore for synchronized with cmd thread */

    void * tac_handle;
    fingerprint_state_t finger_state;
    fingerprint_press_state_t finger_press_state;
	
    uint32_t current_gid;
	
    int32_t sysfs_fd;
    int32_t image_mode;
    int32_t touch_mode;
    char sensor_name[32];
    int32_t fb_blank;
    uint16_t enroll_remaining;
    uint16_t recovery_count;
    uint16_t state_notified;
    struct timeval enrol_timeout;
    struct timeval idle_timeout;
	bool navigation_restart;
    bool disable_sensor;
    bool update_pending;
    bool selftest_pending;
    bool kpi_enabled;
    bool saveimage_enabled;
} fingerprint_data_t;

