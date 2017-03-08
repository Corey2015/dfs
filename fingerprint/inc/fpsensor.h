#ifndef FPSENSOR_COMMON_H
#define FPSENSOR_COMMON_H

#include <limits.h>
#include <unistd.h>
#include <sys/time.h>
#include <cutils/properties.h>
#include "fpsensor_l.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <linux/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <hardware/fingerprint.h>
#include "fingerprint_common.h"
#include "auth.h"

#define FINGERPRINT_SENSOR_PATH "/dev/dfs0"

#define MAX_NBR_TEMPLATES 5
#define QC_AUTH_SEC_APP_NAME_LEN      32

#define NAME_STR_LENGTH        256

#define  FPSENSOR_ERROR_OK                       (0)
#define  FPSENSOR_ERROR_GENERAL            (1)
#define  FPSENSOR_ERROR_MEMORY             (2)
#define  FPSENSOR_ERROR_PARAMETER          (3)
#define  FPSENSOR_ERROR_HANDLE             (4)
#define  FPSENSOR_ERROR_RANGE              (5)
#define  FPSENSOR_ERROR_TIMEOUT            (6)
#define  FPSENSOR_ERROR_STATE              (7)
#define  FPSENSOR_ERROR_APP_NOT_FOUND      (8)
#define  FPSENSOR_ERROR_NO_RESPONSE        (9)
#define  FPSENSOR_ERROR_DUPLICATE          (10)

#define fpsensor_malloc(s)        malloc(s)
#define fpsensor_free(p)          free(p)
#define fpsensor_memcpy(s1,s2,n)  memcpy(s1,s2,n)
#define fpsensor_memset(s,c,n)    memset(s,c,n)
#define fpsensor_memcmp(s1,s2,n)  memcmp(s1,s2,n)
#define fpsensor_strlen(p)        strlen(p)
#define KEYMASTER_SET_ENCAPSULATED_KEY 100

#define DFS747_SENSOR_ROWS        (64)
#define DFS747_SENSOR_COLS        (144)

typedef enum {
    FINGER_DETECT_ERROR = -1,
    FINGER_DETECT_PRESENT = 1,
    FINGER_DETECT_LOST,
    FINGER_DETECT_AGAIN,
} finger_detect_type;

#define CALI_DATA_MAGIC 	0x43414c49
#define DETECT_DATA_MAGIC 	0x44455444
#define IMG_DATA_MAGIC 		0x494d4744


typedef struct
{
	int32_t calibration_magic_num;//0x43414c49
	int32_t img_data_offset;
	int32_t detect_magic_num;//0x44455444
	int16_t detect_cds_offset;
	int16_t detect_thrshld;
	int16_t suspend_interval;

	int32_t img_magic_num;//0x494d4744
	int16_t img_cds_offset;
	int16_t img_thrshld;
	uint8_t bkg_img[DFS747_SENSOR_ROWS * DFS747_SENSOR_COLS];
} dfs_calibration_t;

#define TMPL_INFO_MAGIC 	0x54494e46
typedef struct
{
	int32_t magic_num;
	int32_t ID;
	int16_t state;
	uint16_t count;
} template_info_t;

typedef struct
{
  uint8_t *pImg_feature_data;
  uint8_t *pRaw_imgbuf;
  uint16_t	width;
  uint16_t	height;
  dfs_calibration_t dfs_cal;
  uint8_t *	pFeature_data[MAX_NBR_TEMPLATES]; //指纹特征数据指针
  template_info_t tmpl_info[MAX_NBR_TEMPLATES];
  uint8_t template_state[MAX_NBR_TEMPLATES];//0xaa --avilabe 0x55 -- invalid
  uint8_t enroll_fid;
  char user_tpl_storage_path[128];
  uint32_t path_len;
  uint64_t auth_id;
  int32_t last_match_ID;
  int32_t navigation_count;  
  int empty_avgValue;
  int empty_scoreValue;
  bool enrolling;  
  bool verifying;  
  bool navigating;
} fpsensor_handle_internal_t;

typedef struct
{
    int8_t score;
    int8_t result;
    int8_t fid;
} fpsensor_identify_result_t;
#endif /* FPSENSOR_COMMON_H */
