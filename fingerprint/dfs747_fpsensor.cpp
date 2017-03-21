#define LOG_TAG "FingerprintSensor"

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
#ifdef TAC_TIME_MEASUREMENTS
#include <sys/time.h>
#endif
#include "fpsensor.h"
#include "fp_api.h"         // 新算法接口
#include "fps_driver.h"

// for new version code
#include "fps.h"
#include "fps_control.h"
//

#define use fp_algo
#define MAX_PATH_LEN 256
//--------- 算法所需资源相关配置，不能修改 ----------------------------------------
#define MAX_WIDTH		(120)
#define MAX_HEIGHT		(120)
#define MAX_DBS_FEATURE	(1024*1024)
#define MAX_ONE_FEATURE	(7*1024+MAX_WIDTH*MAX_HEIGHT)

#define SIMILARITY_VALUE 70 //large then this will treat as duplicate
#define	MATCH_SCORE_VALUE 	0.25	// 比对成功阈值 >=此值表示比对成功
#define	UPDATE_SCORE_VALUE 	0.50	// update阈值 >=此值表示should do update
// --------- 算法所需资源相关配置，不能修改  end ----------------------------------
#define MSECS_PER_SECOND (1000)
#define USECS_PER_SECOND (1000000)

#define DFS747_CDS_SEARCH_START (0x01FF)
#define DFS747_CDS_SEARCH_STEP  (1)
#define DFS747_CDS_SEARCH_COUNT ((DFS747_CDS_SEARCH_START + 1) / DFS747_CDS_SEARCH_STEP)

#define IMG_DATA_OFFSET 0x20

#define DETECT_WINDOW_COLS    (8)
#define DETECT_WINDOW_ROWS    (8)

extern fp_core_t fp_core;

extern int fps_multiple_read(const int     fd,
                  const uint8_t *addr,
                  uint8_t       *data,
                  const uint8_t len);
extern int fps_multiple_write(const int     fd,
                   const uint8_t *addr,
                   const uint8_t *data,
                   const uint8_t len);

extern int fps_get_one_image(const int      fd,
                  const uint32_t img_width,
                  const uint32_t img_height,
                  const uint32_t dummy_pix,
                  uint8_t        *img);

extern int fps_single_read(const int     fd,
                const uint8_t addr,
                uint8_t       *data);

extern int fps_single_write(const int     fd,
                 const uint8_t addr,
                 const uint8_t data);

extern int fps_switch_mode(const int fd,
                const int mode_new,
                int       *mode_old);

extern int fps_search_detect_window(const int      fd,
           const uint8_t  col_scan_begin,
           const uint8_t  col_scan_end,
           const uint32_t repeat_times,
           uint16_t       *extra_cds_offset,
           uint8_t        *row_scan_begin,
           uint8_t        *row_scan_end,
           double         *win_dev);
extern int fps_search_detect_window_from_bkgnd_image(const uint8_t col_scan_begin,
                                                     const uint8_t col_scan_end,
                                                     uint8_t       *row_scan_begin,
                                                     uint8_t       *row_scan_end,
                                                     uint16_t      *extra_cds_offset,
                                                     double        *win_dev);

extern int fps_search_detect_threshold(const int      fd,
                            const uint16_t cds_offset,
                            const uint8_t  upper,
                            const uint8_t  lower,
                            const double   sleep_us,
                            const uint32_t scan_limit,
                            const int      int_enable,
                            uint8_t        *detect_th);
extern int fps_enable_tgen(const int fd,
                const int enable);
extern uint8_t find_otsu_th(const uint8_t  *img,
             const uint32_t size);
extern void find_pixel_range(const uint8_t  *img,
                 const uint32_t size,
                 uint8_t        *pix_min,
                 uint8_t        *pix_max);
extern int dump_register(int dev_fd);
//add by corey for 747B
extern int fps_scan_detect_event(const int      fd,
                      const uint8_t  detect_th,
                      const uint16_t cds_offset,
                      const double   sleep_us,
                      const int      int_enable,
                      const int      cal_detect);
extern int fps_search_detect_line(const int      fd,
                       const uint8_t  col_scan_begin,
                       const uint8_t  col_scan_end,
                       const uint8_t  row_scan_begin,
                       const uint8_t  row_scan_end,
                       const uint32_t repeat_times,
                       uint16_t       *extra_cds_offset,
                       uint8_t        *row_found,
                       double         *row_dev);
extern double get_elapsed_ms(const struct timeval *start,
                 const struct timeval *stop);
extern int fps_search_cds_offset(const int      fd,
                      const uint8_t  detect_th,
                      const uint16_t upper,
                      const uint16_t lower,
                      const double   sleep_us,
                      const uint32_t scan_limit,
                      const int      int_enable,
                      uint16_t       *cds_offset);
//end
static int write_calibration_data(const char* path,unsigned char *buf,int size);
static int read_calibration_data(const char* path,unsigned char *buf,int size);
static int setup_calibrate_regs(int32_t dev_fd,dfs_calibration_t dfs_cal);
static int calibrate_detect(int32_t dev_fd,dfs_calibration_t *dfs_cal);
static int calibrate_image(int32_t dev_fd,dfs_calibration_t *dfs_cal);
static int write_template_info(const char* path,unsigned char *buf,int size);
static int read_template_info(const char* path,unsigned char *buf,int size);

//star for new version code
static fps_handle_t* fps_handle_init(int dev_fd);
//static int init_sensor(fps_handle_t *handle);
//end for new version code
////////////////////////////////////////////////
//
//Global Variables
//
//star for new version code
fps_handle_t *fps_handle;
//end for new version code
static uint16_t     det_cds_offset       = 0;
static uint8_t      det_detect_th        = 0;
static double       det_sleep_us         = 0.0;
static uint16_t     det_extra_cds_offset = 0;
static uint32_t     recal_secs           = 30;
static uint32_t     detect_cnt           = 0;

//new version code function
static fps_handle_t* fps_handle_init(int dev_fd){
  int sensor_size;
  fps_handle_t *handle;

  handle = (fps_handle_t *) malloc(sizeof(fps_handle_t));
  if (handle == NULL){
    goto init_error;
  }
  handle->fd = dev_fd;
  handle->chip_id = F747B_CHIP_ID;
  switch (handle->chip_id) {
    case F747B_CHIP_ID :
      handle->sensor_width    = 144;
      handle->sensor_height   = 64;
      handle->latency         = 1;
      handle->power_config    = FPS_POWER_CONFIG_1V8_3V3;
      handle->img_cal_method  = 2;
      handle->det_cal_method  = 4;
      handle->scan_det_method = 1;
      break;

    default :
      goto init_error;
  }
  sensor_size = handle->sensor_width * handle->sensor_height;

  handle->bkgnd_img = (uint8_t *) malloc(sizeof(uint8_t) * sensor_size);
  if (handle->bkgnd_img == NULL) {
      goto init_error;
  }
  return handle;

init_error:
  if (handle != NULL) {
    if (handle->bkgnd_img != NULL) {
      free(handle->bkgnd_img);
    }
    free(handle);
  }

  return NULL;

}

//End for new version code
struct tmpl_sort {
  uint8_t * pFeature_data;
  template_info_t tmpl_info;
};


static uint64_t get_64bit_rand() {
    // This should use a cryptographically-secure random number generator like arc4random().
    // It should be generated inside of the TEE where possible. Here we just use something
    // very simple.
    ALOGD("----------------> %s ----------------->", __FUNCTION__);
    uint64_t r = (((uint64_t)rand()) << 32) | ((uint64_t)rand());
    return r != 0 ? r : 1;
}

int dfs747_begin_enrol(void *pHandle)
{
  int retval = FPSENSOR_ERROR_OK;
  int i;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

  ALOGD("%s begin", __func__);

  if(tac_handle->enrolling == true)
  {
	  ALOGD("already begin enrol,do nothing");
	  return retval;
  }

  tac_handle->enroll_fid = -1;
  for(i=0;i<MAX_NBR_TEMPLATES;i++)
    {
        if(tac_handle->template_state[i] == 0xaa)
           continue;
        tac_handle->enroll_fid = i;
        break;
    }
    if((tac_handle->enroll_fid >= 0) && (tac_handle->enroll_fid < MAX_NBR_TEMPLATES))
    {
        ALOGD("%s get fid %d", __func__,tac_handle->enroll_fid);
        tac_handle->enrolling = true;
        #ifdef use fp_algo
          tac_handle->pFeature_data[tac_handle->enroll_fid] = (uint8_t*) malloc(MAX_DBS_FEATURE);
        #endif

        retval = FPSENSOR_ERROR_OK;
    }
    else retval = FPSENSOR_ERROR_GENERAL;

  return retval;
}

int dfs747_enrol(void *pHandle, void *enroll_data)
{
  int retval = FPSENSOR_ERROR_OK;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;


  struct timeval stop, start, delta;
  int time = 0;
  int state[3]={0};

  ALOGD("%s begin", __func__);

  if(tac_handle->enrolling == false)
  {
	  ALOGE("begin enrol not call,return error");
	  return -FPSENSOR_ERROR_STATE;
  }

  if((tac_handle->enroll_fid < 0) || (tac_handle->enroll_fid >= MAX_NBR_TEMPLATES))
  {
	  ALOGE("enroll_fid is out_of_range,return error");
	  return -FPSENSOR_ERROR_PARAMETER;
  }

  gettimeofday(&start, NULL);

  retval = fp_core.FeatureExtract(tac_handle->pRaw_imgbuf,tac_handle->width,tac_handle->height,tac_handle->pImg_feature_data,NULL);
  if(0 != retval ){
      return -FPSENSOR_ERROR_GENERAL;
  }

  retval = fp_core.FeatureEnroll(tac_handle->pImg_feature_data,tac_handle->pFeature_data[tac_handle->enroll_fid],(void*)state);
  if(0 != retval){
      return -FPSENSOR_ERROR_MEMORY;
  }

  if(state[2]> SIMILARITY_VALUE){
      ALOGE("state[2] %d is large then %d,report duplicate",state[2],SIMILARITY_VALUE);
      return -FPSENSOR_ERROR_DUPLICATE;    // 返回重合度太高
  }

  gettimeofday(&stop, NULL);

  timersub(&stop, &start, &delta);
  time = delta.tv_sec * USECS_PER_SECOND + delta.tv_usec;
  ALOGE("Enrol took %d ms", time / MSECS_PER_SECOND);

  ALOGD("%s end", __func__);

  return retval;
}

int dfs747_end_enrol(void *pHandle)
{
  int retval = FPSENSOR_ERROR_OK;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

  ALOGD("%s begin", __func__);

  if(tac_handle->enrolling == false)
  {
	  ALOGD("already end enrol,do nothing");
	  return retval;
  }
  tac_handle->enrolling = false;

#ifdef use fp_algo
  //fp_end_enroll(tac_handle->pFeature_data[tac_handle->enroll_fid]);
#endif
  ALOGD("%s end, retval: %d", __func__, retval);

  return retval;
}


void bubble_sort(struct tmpl_sort *arr, int len)
{
	int i, j;
	struct tmpl_sort temp;
	struct tmpl_sort a1;
	struct tmpl_sort a2;

	for (i = 0; i < len - 1; i++)
	{
		for (j = 0; j < len - 1 - i; j++)
		{
			a1 = arr[j];
			a2 = arr[j + 1];
			if (a1.tmpl_info.count < a2.tmpl_info.count) {
				temp = arr[j];
				arr[j] = arr[j + 1];
				arr[j + 1] = temp;
			}
		}
	}
}


int dfs747_identify(void *pHandle, const uint8_t* nonce, void *identify_data)
{
  int retval = FPSENSOR_ERROR_OK;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

  struct tmpl_sort sort[MAX_NBR_TEMPLATES];
  struct timeval stop, start, delta;
  int time = 0;
  int i;
  int len = 0;
  float	fSimilarity;
  int updateFlag=0;
  ALOGD("%s begin", __func__);

  retval = fp_core.FeatureExtract(tac_handle->pRaw_imgbuf,tac_handle->width,tac_handle->height,tac_handle->pImg_feature_data,NULL);

  ALOGD("FeatureExtract = %d",retval);

  if(0 != retval ){
     return -FPSENSOR_ERROR_GENERAL;
  }

  gettimeofday(&start, NULL);
  for(i=0;i<MAX_NBR_TEMPLATES;i++)
    {
        if((tac_handle->template_state[i] == 0xaa) && (tac_handle->pFeature_data[i]))
        {
            sort[len].pFeature_data = tac_handle->pFeature_data[i];
            sort[len].tmpl_info = tac_handle->tmpl_info[i];
            if(tac_handle->last_match_ID == i)
                sort[len].tmpl_info.count = 0xFFFF;
            len++;
        }
    }

  bubble_sort(sort,len);
#ifdef use fp_algo
  for(i=0;i < len;i++)
    {
        retval = fp_core.FeatureMatch(tac_handle->pImg_feature_data, sort[i].pFeature_data, &fSimilarity, (void *)&updateFlag);
        ALOGD("match:fSimilarity:%f retval:%d,updateFlag:%d",fSimilarity,retval,updateFlag);
        if(0 != retval){
            fSimilarity = 0.0;
        }

        ((fpsensor_identify_result_t *)identify_data)->score = 100 * fSimilarity;
        if(fSimilarity >= MATCH_SCORE_VALUE)
          {
              ((fpsensor_identify_result_t *)identify_data)->fid = sort[i].tmpl_info.ID + 1;
              tac_handle->last_match_ID = sort[i].tmpl_info.ID;
              if(fSimilarity > UPDATE_SCORE_VALUE) {
                  tac_handle->enroll_fid = sort[i].tmpl_info.ID;
                  ((fpsensor_identify_result_t *)identify_data)->result = 2;
              }else
                  ((fpsensor_identify_result_t *)identify_data)->result = 1;
              ALOGD("match! templateID:%d  verify result:%d",sort[i].tmpl_info.ID,
                      ((fpsensor_identify_result_t *)identify_data)->result);
              tac_handle->tmpl_info[sort[i].tmpl_info.ID].count++;
              write_template_info(tac_handle->user_tpl_storage_path,(unsigned char *)tac_handle->tmpl_info,sizeof(tac_handle->tmpl_info));
              break;
          }
          else {
              ((fpsensor_identify_result_t *)identify_data)->result = 0;
              ALOGD("Not match!i=%d templateID:%d  verify result:%d",i,sort[i].tmpl_info.ID,((fpsensor_identify_result_t *)identify_data)->score);
          }
    }
#endif

  gettimeofday(&stop, NULL);
  timersub(&stop, &start, &delta);
  time = delta.tv_sec * USECS_PER_SECOND + delta.tv_usec;
  ALOGE("Identify took %d ms", time / MSECS_PER_SECOND);

  ALOGD("%s end", __func__);

  return 0;
}

int fpsensor_save_bitmap(const char* FilePath, unsigned char *pData,
		int row, int colume)
{
	/*Check parameter*/
    int fd = 0;
    unsigned char grayBitmapHeader[1078] = { 0 };
	unsigned char pad[4] = {0};
    int colume_t = 0;
    int row_t = 0;
    int i =0;
	if (NULL == pData)
	{
		return -1;
	}
	fd = open(FilePath,O_RDWR| O_CREAT,0777);
	if (0 == fd)
	{
		return -1;
	}

	colume_t = (colume + 3) & 0xFC;
	row_t = (row + 3) & 0xFC;
	grayBitmapHeader[0] = 0x42;
	grayBitmapHeader[1] = 0x4d;
	grayBitmapHeader[2] = 0x36;
	grayBitmapHeader[3] = 0x28;
	grayBitmapHeader[4] = 0x00;
	grayBitmapHeader[5] = 0x00;
	grayBitmapHeader[6] = 0x00;
	grayBitmapHeader[7] = 0x00;
	grayBitmapHeader[8] = 0x00;
	grayBitmapHeader[9] = 0x00;
	grayBitmapHeader[10] = 0x36;
	grayBitmapHeader[11] = 0x04;
	grayBitmapHeader[12] = 0x00;
	grayBitmapHeader[13] = 0x00;
	grayBitmapHeader[14] = 0x28;
	grayBitmapHeader[15] = 0x00;
	grayBitmapHeader[16] = 0x00;
	grayBitmapHeader[17] = 0x00;

	grayBitmapHeader[18] = (colume_t & 0xFF);
	grayBitmapHeader[19] = (colume_t >> 8 & 0xFF);
	grayBitmapHeader[20] = (colume_t >> 16 & 0xFF);
	grayBitmapHeader[21] = (colume_t >> 24 & 0xFF);

	grayBitmapHeader[22] = (row_t & 0xFF);
	grayBitmapHeader[23] = (row_t >> 8 & 0xFF);
	grayBitmapHeader[24] = (row_t >> 16 & 0xFF);
	grayBitmapHeader[25] = (row_t >> 24 & 0xFF);

	grayBitmapHeader[26] = 0x01;
	grayBitmapHeader[27] = 0x00;

	grayBitmapHeader[28] = 0x08;
	grayBitmapHeader[29] = 0x00;

	for (i = 0; i < 256; i++)
	{
		grayBitmapHeader[54 + i * 4] = i;
		grayBitmapHeader[54 + i * 4 + 1] = i;
		grayBitmapHeader[54 + i * 4 + 2] = i;
	}
	write(fd,(char *) grayBitmapHeader, 1078);


	/*Be careful , bitmap save turn is buttom to top*/
	for (i = 0; i < row; i++)
	{
		write(fd,(char *) (pData + (row - i - 1) * colume),
				colume);
		if(colume_t > colume)
			write(fd,(char *)pad, colume_t - colume);
	}
	close(fd);
	return 0;
}

int32_t dfs747_sensor_init(fingerprint_data_t* device)
{
	fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) device->tac_handle;

	int32_t status = 0;
	struct timeval ts_start;
	struct timeval ts_current;
	struct timeval ts_delta;
	int delta_us = 0;
    //int mode_old;

	if (device->kpi_enabled) {
		gettimeofday(&ts_start, NULL);
	}

  //for new version
  fps_handle = fps_handle_init(device->sysfs_fd);
  //ALOGD("init test %d,%d",fps_handle->sensor_width,fps_handle->sensor_height);
  if (fps_handle == NULL){
    ALOGD("fps_handle_init failed ");
    return -1;
  }
	status = fps_init_sensor(fps_handle);
  ALOGD("init status = %d",status);
  //end

	if (device->kpi_enabled) {
		gettimeofday(&ts_current, NULL);
		timersub(&ts_current, &ts_start, &ts_delta);
		delta_us = TIME_IN_US(ts_delta);
		ALOGD("KPI fingerprintsensor init: %d ms",delta_us / 1000);
	}

	return status;
}

int32_t dfs747_sensor_deinit(fingerprint_data_t* device)
{
	int status;
	struct timeval ts_start;
	struct timeval ts_current;
	struct timeval ts_delta;
	int delta_us = 0;

	if (device->kpi_enabled) {
		gettimeofday(&ts_start, NULL);
	}

	//close_sensor(device->sysfs_fd);

	if (device->kpi_enabled) {
		gettimeofday(&ts_current, NULL);
		timersub(&ts_current, &ts_start, &ts_delta);
		delta_us = TIME_IN_US(ts_delta);
		ALOGD("KPI fingerprintsensor deinit: %d ms",delta_us / 1000);
	}

	return status;
}

int setup_calibrate_regs(int32_t dev_fd,dfs_calibration_t dfs_cal)
{
    int status;
    uint8_t        addr[4];
    uint8_t        data[4];

	// Fill the result to the registers
	addr[0] = DFS747_REG_DET_CDS_CTL_0;
	addr[1] = DFS747_REG_DET_CDS_CTL_1;
	addr[2] = DFS747_REG_V_DET_SEL;

	status = fps_multiple_read(dev_fd, addr, data, 3);

	data[0] = (data[0] & 0x7F) | ((uint8_t) ((dfs_cal.detect_cds_offset & 0x0100) >> 1));
	data[1] = (uint8_t) (dfs_cal.detect_cds_offset & 0x00FF);
	data[2] = dfs_cal.detect_thrshld;

	status = fps_multiple_write(dev_fd, addr, data, 3);

	// Set suspend interval
	addr[0] = DFS747_REG_SUSP_WAIT_F_CYC_H;
	addr[1] = DFS747_REG_SUSP_WAIT_F_CYC_L;

	data[0] = (uint8_t) ((dfs_cal.suspend_interval & 0xFF00) >> 8);
	data[1] = (uint8_t) ((dfs_cal.suspend_interval & 0x00FF) >> 0);

	status = fps_multiple_write(dev_fd, addr, data, 2);

	addr[0] = DFS747_REG_IMG_CDS_CTL_0;
	addr[1] = DFS747_REG_IMG_CDS_CTL_1;
	addr[2] = DFS747_REG_IMG_PGA1_CTL;

	// Set CDS Offset and PGA Gain
	status = fps_multiple_read(dev_fd, addr, data, 3);

	data[0] = ((uint8_t) ((dfs_cal.img_cds_offset & 0x0100) >> 1)) | (data[0] & 0x7F);
	data[1] =  (uint8_t)  (dfs_cal.img_cds_offset & 0x00FF);
	data[2] = dfs_cal.img_thrshld & 0x0F;

	status = fps_multiple_write(dev_fd, addr, data, 3);
    return 0;
}

int calibrate_detect(int32_t dev_fd,dfs_calibration_t *dfs_cal)
{
  const int DEFAULT_FRAMES_TO_SUSPEND = 2;
  const int DEFAULT_DETECT_WIDTH              = 8;
  const int DEFAULT_DETECT_HEIGHT             = 8;
  int status = 0;
  int detect_th;
  int det_width;
  int det_height;
  int frms_to_susp;
  int cds_offset;
  int det_col_begin;
  int det_col_end;
  int det_row_begin;
  int det_row_end;
  int mode_old;
  struct timeval  start_time;
  struct timeval  stop_time;
  double elapsed;

  det_width    = DEFAULT_DETECT_WIDTH;
  det_height   = DEFAULT_DETECT_HEIGHT;
  frms_to_susp = DEFAULT_FRAMES_TO_SUSPEND;

  ALOGD("Detect calibrating...");

  (void) gettimeofday(&start_time, NULL);

  status = fps_switch_mode(fps_handle, FPS_DETECT_MODE, &mode_old);
  if (status < 0) {
      return status;
  }

  status = fps_detect_calibration(fps_handle,
                                  det_width,
                                  det_height,
                                  frms_to_susp);

  status = fps_switch_mode(fps_handle, mode_old, NULL);
  if (status < 0) {
      return status;
  }

  (void) gettimeofday(&stop_time, NULL);
  elapsed = get_elapsed_ms(&start_time, &stop_time);


  if (status < 0) {
      printf("    Failed!\n");
  } else {
      // Get final detect window
      (void) fps_get_sensing_area(fps_handle, FPS_DETECT_MODE,
                                  &det_col_begin, &det_col_end,
                                  &det_row_begin, &det_row_end);

      // Get final Detect Threshold
      (void) fps_get_sensor_parameter(fps_handle,
                                      FPS_PARAM_DETECT_THRESHOLD,
                                      &detect_th);

      // Get final CDS Offset
      (void) fps_get_sensor_parameter(fps_handle,
                                      (FPS_DETECT_MODE | FPS_PARAM_CDS_OFFSET_1),
                                      &cds_offset);

      ALOGD("    Successful!");
      ALOGD("    Window (CB,RB):(CE,RE) = (%0d,%0d):(%0d,%0d)",
             det_col_begin, det_row_begin, det_col_end, det_row_end);
      ALOGD("    Detect Threshold       = 0x%02X", detect_th);
      ALOGD("    CDS Offset 1           = 0x%03X", cds_offset);
      ALOGD("    Time Elapsed           = %0.3f ms", elapsed);
  }

  dfs_cal->detect_magic_num = DETECT_DATA_MAGIC;
  dfs_cal->detect_cds_offset = cds_offset;
  dfs_cal->detect_thrshld= detect_th;
  //dfs_cal->suspend_interval= det_frame;
  det_detect_th        = detect_th;
  det_cds_offset       = cds_offset;
  //det_sleep_us         = sleep_us;
  //det_extra_cds_offset = extra_cds_offset;
  /*
  ALOGD("    Done!\n");
  ALOGD("\n");
  ALOGD("    Selected Row (Begin/End) = %0d/%0d\n", row_begin, row_end);
  ALOGD("    Detect Threshold         = 0x%02X\n",  detect_th);
  ALOGD("    CDS Offset               = 0x%03X\n",  cds_offset);
  ALOGD("\n");
  ALOGD("    Time elapsed = %0.3f ms\n", elapsed);
  */
  return status;
}

int calibrate_image(int32_t dev_fd,dfs_calibration_t *dfs_cal)
{
  int status = 0;
  int mode_old;
  int img_width;
  int img_height;
  int frms_to_avg;
  struct timeval  start_time;
  struct timeval  stop_time;
  int cds_offset;
  int pga_gain;
  double elapsed;
  int img_size;
  ALOGD("image   Calibrating...\n");

  (void) gettimeofday(&start_time, NULL);

  status = fps_switch_mode(fps_handle, FPS_IMAGE_MODE, &mode_old);
  if (status < 0) {
      return status;
  }

  img_width   = fps_handle->sensor_width;
  img_height  = fps_handle->sensor_height;
  frms_to_avg = 1;
  img_size = img_width * img_height;

  status = fps_image_calibration(fps_handle,
                                 img_width,
                                 img_height,
                                 frms_to_avg);

  status = fps_switch_mode(fps_handle, mode_old, NULL);
  (void) gettimeofday(&stop_time, NULL);
  elapsed = get_elapsed_ms(&start_time, &stop_time);
  if (status < 0) {
      ALOGD("calibrate image failed");
      return status;
  }else{
    // Get final CDS offset
    (void) fps_get_sensor_parameter(fps_handle,
                                    (FPS_IMAGE_MODE | FPS_PARAM_CDS_OFFSET_1),
                                    &cds_offset);

    // Get final PGA gain
    (void) fps_get_sensor_parameter(fps_handle,
                                    (FPS_IMAGE_MODE | FPS_PARAM_PGA_GAIN_1),
                                    &pga_gain);
    memcpy(dfs_cal->bkg_img, fps_handle->bkgnd_img, img_size);
    dfs_cal->img_cds_offset = cds_offset;
    dfs_cal->img_thrshld = pga_gain;
    ALOGD("    Done!\n");
    ALOGD("\n");
    ALOGD("    CDS Offset = 0x%03X\n", cds_offset);
    ALOGD("    PGA Gain   = 0x%02X\n", pga_gain);
    ALOGD("\n");
    ALOGD("    Time elapsed = %0.3f ms\n", elapsed);
  }


  return status;


}

static int write_calibration_data(const char* path,unsigned char *buf,int size)
{
  mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
  int fd;
  unsigned char *wbuf = NULL;
  dfs_calibration_t *cali_data = (dfs_calibration_t *)buf;
  int len;
  int retval;
  char calibration_file[PATH_MAX];

  if (path == NULL) {
	ALOGE("%s calibrator_data storage calibration_file is null", __func__);
	return FPSENSOR_ERROR_GENERAL;
  }

  if(size != sizeof(dfs_calibration_t)){
	ALOGE("%s size should be dfs_calibration_t", __func__);
	return FPSENSOR_ERROR_PARAMETER;
  }
  snprintf(calibration_file,PATH_MAX,"%s/dfs_cal.bin",path);
  fd = open(calibration_file, O_WRONLY | O_CREAT | O_TRUNC, mode);

  if (fd < 0) {
	ALOGE("%s failed to open file %s, errno %d", __func__, calibration_file, errno);
	return FPSENSOR_ERROR_GENERAL;
  }

    len = sizeof(*cali_data) + cali_data->img_data_offset;
    wbuf = (unsigned char *) malloc(len);
    if(wbuf == NULL) {
        ALOGE("fail to malloc buffers for calibrator_data write");
        return -ENOMEM;
    }

    memset(wbuf,0,len);
	*(int32_t *)wbuf  = CALI_DATA_MAGIC;
	*((int32_t *)(wbuf + 4)) = cali_data->img_data_offset;
	*((int32_t *)(wbuf + 8)) = DETECT_DATA_MAGIC;
	*(int16_t *)(wbuf + 12) = cali_data->detect_cds_offset;
	*(int16_t *)(wbuf + 14)= cali_data->detect_thrshld;
	*(int16_t *)(wbuf + 16)= cali_data->suspend_interval;

 	*((int32_t *)(wbuf + cali_data->img_data_offset)) = IMG_DATA_MAGIC;
	*(int16_t *)(wbuf + cali_data->img_data_offset + 4) = cali_data->img_cds_offset;
	*(int16_t *)(wbuf + cali_data->img_data_offset + 6) = cali_data->img_thrshld;
	memcpy((wbuf + cali_data->img_data_offset + 8), cali_data->bkg_img , DFS747_SENSOR_SIZE);

    retval = write(fd, wbuf, len);
    free(wbuf);
    close(fd);
    if (retval == -1) {
	    ALOGE("%s failed to write file %s, errno %d", __func__, calibration_file, errno);
    	return FPSENSOR_ERROR_GENERAL;
    } else if (retval != len) {
	  ALOGE("%s failed to write full file %s, errno %d", __func__, calibration_file, errno);
	  return FPSENSOR_ERROR_GENERAL;
	} else {
	  ALOGD("%s Succesfully write calibrator_data %s", __func__, calibration_file);
	}

  return FPSENSOR_ERROR_OK;
}

static int read_calibration_data(const char* path,unsigned char *buf,int size)
{
  mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
  int len = 0;
  int fd;
  struct stat fileStat;
  int32_t retval;
  unsigned char *rbuf = NULL;
  dfs_calibration_t *cali_data = (dfs_calibration_t *)buf;
  char calibration_file[PATH_MAX];

  if (path == NULL) {
	ALOGE("%s calibrator_data storage calibration_file is null", __func__);
	return FPSENSOR_ERROR_PARAMETER;
  }

  if(size != sizeof(dfs_calibration_t)){
	ALOGE("%s size should be dfs_calibration_t", __func__);
	return FPSENSOR_ERROR_PARAMETER;
  }

  snprintf(calibration_file,PATH_MAX,"%s/dfs_cal.bin",path);
  retval = stat(calibration_file, &fileStat);

  if (retval < 0) {
	ALOGD("stat returned error: %d", retval);
	/* File does not exist, it needs to be created */
	return FPSENSOR_ERROR_GENERAL;
  }

  len = fileStat.st_size;
  if(len < sizeof(*cali_data) + IMG_DATA_OFFSET) {
	ALOGD("file size to small: %d", len);
	return FPSENSOR_ERROR_GENERAL;
  }
  fd = open(calibration_file, O_RDONLY, mode);
  if (fd <= 0) {
	  ALOGE("%s failed to open file %s ret %d", __func__, calibration_file, fd);

	  return FPSENSOR_ERROR_GENERAL;
  	}

    rbuf = (unsigned char *) malloc(len);

	retval = read(fd, rbuf, len);
	close(fd);
	if (retval != len) {
	  ALOGE("%s failed to read full size of file %s ", __func__, calibration_file);
	  return FPSENSOR_ERROR_GENERAL;
	} else {
	  ALOGD("%s Succesfully read calibrator_data %s", __func__, calibration_file);
	}

	if(*((int32_t *)rbuf) != CALI_DATA_MAGIC) {
	  ALOGE("%s calibrate data magic mismatch,rm it", __func__);
	  goto err_file;
	}
	cali_data->img_data_offset = *((int32_t *)(rbuf + 4));
	if(*((int32_t *)(rbuf + 8)) != DETECT_DATA_MAGIC) {
	  ALOGE("%s detect data magic mismatch,rm it", __func__);
	  goto err_file;
	}
	cali_data->detect_cds_offset = *((int16_t *)(rbuf + 12));
	cali_data->detect_thrshld = *((int16_t *)(rbuf + 14));
	cali_data->suspend_interval = *((int16_t *)(rbuf + 16));

 	if(*((int32_t *)(rbuf + cali_data->img_data_offset)) != IMG_DATA_MAGIC) {
	   ALOGE("%s image data magic mismatch,rm it", __func__);
	   goto err_file;
 	}
	cali_data->img_cds_offset = *(int16_t *)(rbuf + cali_data->img_data_offset + 4);
	cali_data->img_thrshld = *(int16_t *)(rbuf + cali_data->img_data_offset + 6);
	memcpy(cali_data->bkg_img , (rbuf + cali_data->img_data_offset + 8), DFS747_SENSOR_SIZE);

  return FPSENSOR_ERROR_OK;
err_file:
  ALOGE("%s rm it", __func__);
  unlink(calibration_file);
  return FPSENSOR_ERROR_GENERAL;
}


static int write_template_info(const char* path,unsigned char *buf,int size)
{
  mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
  int fd;
  unsigned char *wbuf = NULL;
  template_info_t *tmpl_info = (template_info_t *)buf;
  int len;
  int retval;
  int i;
  char tmpl_info_file[PATH_MAX];

  if (path == NULL) {
	ALOGE("%s calibrator_data storage tmpl_info_file is null", __func__);
	return FPSENSOR_ERROR_GENERAL;
  }

  if(size != sizeof(template_info_t)*MAX_NBR_TEMPLATES){
	ALOGE("%s size should be template_info_t", __func__);
	return FPSENSOR_ERROR_PARAMETER;
  }
  snprintf(tmpl_info_file,PATH_MAX,"%s/tmpl_info.dat",path);
  fd = open(tmpl_info_file, O_WRONLY | O_CREAT | O_TRUNC, mode);

  if (fd < 0) {
	ALOGE("%s failed to open file %s, errno %d", __func__, tmpl_info_file, errno);
	return FPSENSOR_ERROR_GENERAL;
  }

    len = sizeof(template_info_t) * MAX_NBR_TEMPLATES;
    ALOGE("len %d",len);
    wbuf = (unsigned char *) malloc(len);
    if(wbuf == NULL) {
        ALOGE("fail to malloc buffers for calibrator_data write");
        return -ENOMEM;
    }

    memset(wbuf,0,len);
    for(i=0;i<MAX_NBR_TEMPLATES;i++) {
        *((int32_t *)(wbuf + 12*i + 0))  = TMPL_INFO_MAGIC;
        *((int32_t *)(wbuf + 12*i + 4))  = tmpl_info[i].ID;
        *((int16_t *)(wbuf + 12*i + 8))  = tmpl_info[i].state;
        *((int16_t *)(wbuf + 12*i + 10)) = tmpl_info[i].count;
	    ALOGE("%s %d count:%d", __func__, i, tmpl_info[i].count);
    }

    retval = write(fd, wbuf, len);
    free(wbuf);
    close(fd);
    if (retval == -1) {
	    ALOGE("%s failed to write file %s, errno %d", __func__, tmpl_info_file, errno);
    	return FPSENSOR_ERROR_GENERAL;
    } else if (retval != len) {
	  ALOGE("%s failed to write full file %s, errno %d", __func__, tmpl_info_file, errno);
	  return FPSENSOR_ERROR_GENERAL;
	} else {
	  ALOGD("%s Succesfully write calibrator_data %s", __func__, tmpl_info_file);
	}

  return FPSENSOR_ERROR_OK;
}

static int read_template_info(const char* path,unsigned char *buf,int size)
{
  mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
  int len = 0;
  int fd;
  int i;
  struct stat fileStat;
  int32_t retval;
  unsigned char *rbuf = NULL;
  template_info_t *tmpl_info = (template_info_t *)buf;
  char tmpl_info_file[PATH_MAX];

  if (path == NULL) {
	ALOGE("%s calibrator_data storage tmpl_info_file is null", __func__);
	return FPSENSOR_ERROR_PARAMETER;
  }

  if(size != sizeof(template_info_t)*MAX_NBR_TEMPLATES){
	ALOGE("%s size should be template_info_t", __func__);
	return FPSENSOR_ERROR_PARAMETER;
  }

  snprintf(tmpl_info_file,PATH_MAX,"%s/tmpl_info.dat",path);
  retval = stat(tmpl_info_file, &fileStat);

  if (retval < 0) {
	ALOGD("stat returned error: %d", retval);
	/* File does not exist, it needs to be created */
	return FPSENSOR_ERROR_GENERAL;
  }

  len = fileStat.st_size;
  if(len < size) {
	ALOGD("file size to small: %d", len);
	return FPSENSOR_ERROR_GENERAL;
  }
  fd = open(tmpl_info_file, O_RDONLY, mode);
  if (fd <= 0) {
	  ALOGE("%s failed to open file %s ret %d", __func__, tmpl_info_file, fd);

	  return FPSENSOR_ERROR_GENERAL;
  	}

    rbuf = (unsigned char *) malloc(len);

	retval = read(fd, rbuf, len);
	close(fd);
	if (retval != len) {
	  ALOGE("%s failed to read full size of file %s ", __func__, tmpl_info_file);
	  return FPSENSOR_ERROR_GENERAL;
	} else {
	  ALOGD("%s Succesfully read calibrator_data %s", __func__, tmpl_info_file);
	}

    for(i=0;i<MAX_NBR_TEMPLATES;i++) {
        if(*((int32_t *)(rbuf + 12*i +0)) != TMPL_INFO_MAGIC) {
          ALOGE("%s template info magic mismatch,rm it", __func__);
          goto err_file;
        }
        tmpl_info[i].ID   = *((int32_t *)(rbuf + 12*i + 4));
        tmpl_info[i].state= *((int16_t *)(rbuf + 12*i + 8));
        tmpl_info[i].count= *((int16_t *)(rbuf + 12*i + 10));
	    ALOGE("%s %d count:%d", __func__, i, tmpl_info[i].count);
    }

  ALOGE("%s success", __func__);
  return FPSENSOR_ERROR_OK;
err_file:
  ALOGE("%s rm it", __func__);
  unlink(tmpl_info_file);
  return FPSENSOR_ERROR_GENERAL;
}

static int get_file_size(const char* path, uint32_t *size)
{
  struct stat fileStat;
  int32_t retval;

  if (path == NULL) {
	return FPSENSOR_ERROR_PARAMETER;
  }

  retval = stat(path, &fileStat);

  if (retval < 0) {
	ALOGD("stat returned error: %d", retval);
	/* File does not exist, it needs to be created */
	return FPSENSOR_ERROR_GENERAL;
  }

  *size = fileStat.st_size;
  return FPSENSOR_ERROR_OK;
}

int get_image(fingerprint_data_t* device, uint8_t *enh_img)
{
	int retval = FPSENSOR_ERROR_OK;
	fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) device->tac_handle;
    int            status     = 0;
    char           *arg       = NULL;
    uint8_t        avg_frame  = 1; //modified by corey avg_frame  = 4
    uint8_t        addr[4];
    uint8_t        data[4];
    int            mode_old   = 0;
    uint8_t        row_begin  = 0;
    uint8_t        row_end    = DFS747_SENSOR_ROWS - 1;
    uint8_t        col_begin  = 0;
    uint8_t        col_end    = DFS747_SENSOR_COLS - 1;
    uint32_t       img_width  = DFS747_SENSOR_COLS;
    uint32_t       img_height = DFS747_SENSOR_ROWS;
    uint32_t       img_size   = (DFS747_SENSOR_COLS * DFS747_SENSOR_ROWS);
    uint8_t        **raw_buf  = NULL;
    uint8_t        **img_buf  = NULL;
    uint8_t        *avg_img   = NULL;
    uint8_t        *fng_img   = NULL;
    uint8_t        *raw_bkg   = NULL;
    double         otsu_mul   = 1.0;
    uint8_t        otsu_th    = 0;
    double         intensity  = 0.0;
    struct timeval start_time;
    struct timeval stop_time;
    double         elapsed    = 0.0;
    time_t         begin_time;
    struct tm      *ptm;
    char           file[CHAR_CNT];
    FILE           *fp        = NULL;
    double         sum        = 0.0;
    double         bkg_avg    = 0.0;
    double         fng_avg    = 0.0;
    double         fng_dr     = 0.0;
    uint32_t       enh_range  = 200;
    uint8_t        pix_min    = 0;
    uint8_t        pix_max    = 0;
    double         contrast   = 0.0;
    double         brightness = 0.0;
    int            f;
    int            i;
    int            j;


    memset(enh_img,0,img_size);
        // Calculate background image average
        sum = 0.0;
        for (i = 0; i < img_size; i++) {
            sum += (double) tac_handle->dfs_cal.bkg_img[i];
        }
        bkg_avg = sum / img_size;

        // Switch to image mode
        status = fps_switch_mode(device->sysfs_fd, DFS747_POWER_DOWN_MODE, &mode_old);
        status = fps_switch_mode(device->sysfs_fd, DFS747_IMAGE_MODE, NULL);
        if (status < 0) {
            goto image_mode_test_error;
        }

        status = gettimeofday(&start_time, NULL);
        if (status < 0) {
            goto image_mode_test_error;
        }

        // Repeat to aquire and average frames
            img_buf = (uint8_t **) malloc(sizeof(uint8_t *) * avg_frame);
            if (img_buf == NULL) {
                status = -1;
                goto image_mode_test_error;
            }

            raw_buf = (uint8_t **) malloc(sizeof(uint8_t *) * avg_frame);
            if (raw_buf == NULL) {
                status = -1;
                goto image_mode_test_error;
            }

            // Create image buffers to average
            for (f = 0; f < avg_frame; f++) {
                raw_buf[f] = (uint8_t *) malloc(img_size + DFS747_DUMMY_PIXELS);
                if (raw_buf[f] == NULL) {
                    status = -1;
                    goto image_mode_test_error;
                }
                img_buf[f] = &raw_buf[f][DFS747_DUMMY_PIXELS];
            }

            // Create an image buffer to store averaged result
            avg_img = (uint8_t *) malloc(img_size);
            if (avg_img == NULL) {
                status = -1;
                goto image_mode_test_error;
            }


            // Create an image buffer for finger only
            fng_img = (uint8_t *) malloc(img_size);
            if (fng_img == NULL) {
                status = -1;
                goto image_mode_test_error;
            }

            for (f = 0; f < avg_frame; f++) {
                // Clear all pending events
                status = fps_single_write(device->sysfs_fd, DFS747_REG_INT_EVENT, 0x00);
                if (status < 0) {
                    goto image_mode_test_error;
                }

                // Turn on TGEN
                status = fps_enable_tgen(device->sysfs_fd, 1);
                if (status < 0) {
                    goto image_mode_test_error;
                }

                // Get a frame
                status = fps_get_one_image(device->sysfs_fd,
                                           img_width,
                                           img_height,
                                           DFS747_DUMMY_PIXELS,
                                           raw_buf[f]);
                if (status < 0) {
                    goto image_mode_test_error;
                }

                // Turn off TGEN
                status = fps_enable_tgen(device->sysfs_fd, 0);
                if (status < 0) {
                    goto image_mode_test_error;
                }
            }

            // Average each frames
            for (i = 0; i < img_size; i++) {
                sum = 0.0;
                for (f = 0; f < avg_frame; f++) {
                    sum += (double) img_buf[f][i];
                }

                avg_img[i] = (uint8_t) (sum / avg_frame);
            }

            // Save (averaged) image
            // sprintf(file, "%s/average.bmp", line);
            // fpsensor_save_bitmap(file, avg_img,DFS747_SENSOR_ROWS,DFS747_SENSOR_COLS);
            // if (status < 0) {
            //     goto image_mode_test_error;
            // }

            // Calculate finger image average
            sum = 0.0;
            for (i = 0; i < img_size; i++) {
                sum += (double) avg_img[i];
            }
            fng_avg = sum / img_size;

#if 0
            // Calculate finger image deviation
            sum = 0.0;
            for (i = 0; i < img_size; i++) {
            }
#endif

            // Extract fingerprint
            for (i = 0; i < img_size; i++) {
                if (avg_img[i] > tac_handle->dfs_cal.bkg_img[i]) {
                    fng_img[i] = 0xFF - (avg_img[i] - tac_handle->dfs_cal.bkg_img[i]);
                } else {
                    fng_img[i] = 0xFF;
                }
            }

            // Calculate dynamic range
            fng_dr = fng_avg - bkg_avg;

            // Save finger image
            // sprintf(file, "%s/finger.bmp", line);
            // fpsensor_save_bitmap(file, fng_img,DFS747_SENSOR_ROWS,DFS747_SENSOR_COLS);
            // if (status < 0) {
            //     goto image_mode_test_error;
            // }

            //modify by corey
	    #if 0
            if (otsu_mul > 0.0) {
                // Find Otsu threshold
                otsu_th = find_otsu_th(fng_img, img_size);
                ALOGD("%s(): Enhancement Th. = %0d\n", __func__, otsu_th);

                for (i = 0; i < img_size; i++) {
                    intensity = (double) fng_img[i];

                    if (intensity > (double) otsu_th) {
                        intensity += ((intensity - (double) otsu_th) * otsu_mul + 0.5);
                        if (intensity > 255.0) {
                            intensity = 255.0;
                        }
                    } else {
                        intensity -= (((double) otsu_th - intensity) * otsu_mul + 0.5);
                        if (intensity < 0.0) {
                            intensity = 0.0;
                        }
                    }

                    enh_img[i] = (uint8_t) intensity;
                }
            }

            if (enh_range > 0) {
                // Find the min. and max. value in this image
                find_pixel_range(enh_img, img_size, &pix_min, &pix_max);
                ALOGD("%s(): (Before) pix_min = %0d, pix_max = %0d\n", __func__, pix_min, pix_max);

                // Use contrast/brightness to enhance the image
                contrast   = ((double) enh_range) / ((double) pix_max - (double) pix_min);
                brightness = (((double) pix_max + (double) pix_min) / 2) * contrast - 128.0;
                ALOGD("%s(): contrast = %0.3f, brightness = %0.3f\n", __func__, contrast, brightness);

                for (i = 0; i < img_size; i++) {
                    intensity = enh_img[i] * contrast - brightness;

                    if (intensity > 255.0) {
                        intensity = 255.0;
                    }
                    else if (intensity < 0.0) {
                        intensity = 0.0;
                    }

                    enh_img[i] = (uint8_t) intensity;
                }

                // Find the min. and max. value in this image
                find_pixel_range(enh_img, img_size, &pix_min, &pix_max);
                ALOGD("%s(): (After) pix_min = %0d, pix_max = %0d\n", __func__, pix_min, pix_max);
            }
            // Save enhanced image
            sprintf(file, "%s/enhanced.bmp", line);
            fpsensor_save_bitmap(file, enh_img,DFS747_SENSOR_ROWS,DFS747_SENSOR_COLS);
            if (status < 0) {
                goto image_mode_test_error;
            }
	    #endif

            //add by corey 2016/05/31
			      sprintf(file, "/data/system/users/0/fpdata/dolfa/%d.bmp",start_time.tv_sec);
            ALOGD("Save_bmp :%s\n",file);
            fpsensor_save_bitmap(file, fng_img,DFS747_SENSOR_ROWS,DFS747_SENSOR_COLS);
            memcpy(enh_img,fng_img,DFS747_SENSOR_ROWS*DFS747_SENSOR_COLS);

            //memcpy(enh_img,avg_img,DFS747_SENSOR_ROWS*DFS747_SENSOR_COLS);
            // Free allocated buffers
            if (fng_img) free(fng_img);
            if (avg_img) free(avg_img);
            for (f = 0; f < avg_frame; f++) {
                if (raw_buf[f]) free(raw_buf[f]);
            }
            if (raw_buf) free(raw_buf);
            if (img_buf) free(img_buf);

            // Display DR
            ALOGD("    Image Dynamic Range = %0.3f\n", fng_dr);

        status = gettimeofday(&stop_time, NULL);
        if (status < 0) {
            goto image_mode_test_error;
        }


        // Switch back to original mode
//        status = fps_switch_mode(device->sysfs_fd, DFS747_POWER_DOWN_MODE, NULL);
//        status = fps_switch_mode(device->sysfs_fd, mode_old, NULL);
        //status = fps_switch_mode(device->sysfs_fd, mode_old, &mode_old);
        if (status < 0) {
            goto image_mode_test_error;
        }

		//fps_single_write(device->sysfs_fd, DFS747_REG_INT_EVENT, 0x00);
        ALOGD("    Done!\n");

    return status;

image_mode_test_error :

    // Free allocated buffers
    if (raw_bkg) free(raw_bkg);
    if (fng_img) free(fng_img);
    if (avg_img) free(avg_img);
    for (f = 0; f < avg_frame; f++) {
        if (raw_buf[f]) free(raw_buf[f]);
    }
    if (raw_buf) free(raw_buf);
    if (img_buf) free(img_buf);

    ALOGD("    Failed!\n");
    ALOGD("\n");

    return status;
}

/**********************************************************************
* 计算差分值
* 输  入: data        图像数据
*        width       图像宽度
*        rowStart    起始行
*        rowLen      待计算的总行数
*        colStart    起始列
*        colLen      待计算的总列数
*
* 输  出: 差分值
**********************************************************************/
int imgGetWaveExt(uint8_t *data,
                  int width,
                  int rowStart,
                  int rowLen,
                  int colStart,
                  int colLen){
    int col=0,row=0;
    int tmp=0;
    int scoreRow=0,scoreCol=0;

    // 针对 行 像素做差分统计
    for (row= rowStart;row < rowStart+rowLen;row ++){
        for (col = colStart;col < colStart+colLen-1;col++){
            tmp = row*width+col;
            scoreRow += abs(data[tmp + 1] - data[tmp]);
        }
    }

    // 针对 列 像素做差分统计
    for (col = colStart;col < colStart+colLen;col++){
        for (row= rowStart;row < rowStart+rowLen-1;row ++){
            tmp = row*width+col;
            scoreCol += abs(data[tmp+width] - data[tmp]);
        }
    }

    if (scoreCol>scoreRow){
        return scoreCol;
    } else {
        return scoreRow;
    }

}


/**********************************************************************
 * 计算平均灰度值
 **********************************************************************/
int averageImage(uint8_t *data,int leng){
    int num=0, i=0;

    for (i=0;i<leng;i++){
        num += (int8_t)data[i];
    }

    num = num / leng;   // 平均灰度

    return num;
}



/***********************************************************************
* 功能简介: 图像质量判断
* 输　  入: fpmat       -- 待处理图像
*		   avgValue    -- 输出的 此图像平均灰度值
*          scoreValue  -- 输出的 此图像差分值
* 输　  出: 0 -- 成功  -1 -- 输入数据出错
***********************************************************************/
int fp_finger_detection(fpsensor_handle_internal_t* tac_handle, int *avgValue, int *scoreValue)
{
   /*
   //deprecated in 747B
    int score=0;

    if(NULL == avgValue || NULL == scoreValue){
        return -FPSENSOR_ERROR_PARAMETER;
    }

    *avgValue = averageImage(tac_handle->pRaw_imgbuf, tac_handle->height * tac_handle->width);

    score = imgGetWaveExt(tac_handle->pRaw_imgbuf,tac_handle->width,0,tac_handle->height,0,tac_handle->width);
    score = score / (tac_handle->height * tac_handle->width);

    *scoreValue = score;

    return 0;
    */
     if(NULL == avgValue || NULL == scoreValue){
        return -FPSENSOR_ERROR_PARAMETER;
    }

	   const uint8_t value = 0xC8;
	    //const uint8_t value = 0xF0;
    uint8_t *pImg = tac_handle->pRaw_imgbuf;
    uint32_t len_img = tac_handle->height * tac_handle->width;

	  int i, cnt;
	  for(i=0, cnt=0; i<len_img; i++) {
		if(pImg[i] < value)
			cnt ++;
	  }
    *scoreValue = 100*cnt/len_img;
    return 0;
}

int32_t dfs747_captureImage(fingerprint_data_t* device)
{
  /*
  static int s_avgValue =-13;
  static int s_scoreValue = 4;
  static bool s_avg_flag = false;
  static bool s_score_flag = false;
  */
  int retval = FPSENSOR_ERROR_OK;
	fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) device->tac_handle;
	struct timeval ts_start;
	struct timeval ts_current;
	struct timeval ts_delta;
	int delta_us = 0;
	uint32_t result = 0;
  uint8_t *buf = NULL;
  int avgValue =0;
  int scoreValue = 0;
  int i = 0;
  //int img_qlty = 0;

	if (device->kpi_enabled) {
		gettimeofday(&ts_start, NULL);
	}

  ALOGD("%s begin", __func__);
  for (i = 0; i < 10; i++) {
    /* code */

    get_image(device,tac_handle->pRaw_imgbuf);
    fp_finger_detection(tac_handle, &avgValue, &scoreValue);
    if(scoreValue > 45){
        //可使用图像
		//add by corey for save image 2016/11/25
		//memset(&tv, 0, sizeof(timeval));
    /*
    memset(line,0,CHAR_CNT);
		time_ptr = localtime(&ts_start.tv_sec);
		sprintf(line, "%d%02d%02d_%02d%02d%02d%04d",
            time_ptr->tm_year + 1900,
            time_ptr->tm_mon + 1,
            time_ptr->tm_mday,
            time_ptr->tm_hour,
            time_ptr->tm_min,
            time_ptr->tm_sec,
            ts_start.tv_usec);
		sprintf(file, "/data/system/users/0/fpdata/dolfa/%s.bin", line);
		 ALOGD("file path is %s",file);
		status = save_bmp(file, tac_handle->pRaw_imgbuf, img_size);
		if (status <0){
			ALOGD("Save file failed!");
		}
		*/
		//end
        ALOGD("image qulity ok");
        retval = FPSENSOR_ERROR_OK;
        break;
    } else {
        // 不可使用图像
        ALOGD("image qulity error");
        retval = FPSENSOR_ERROR_STATE;
        //retval = FPSENSOR_ERROR_OK;
    }
  }

    ALOGD("avgValue =%d scoreValue=%d",avgValue,scoreValue);
	if (device->kpi_enabled) {
		gettimeofday(&ts_current, NULL);
		timersub(&ts_current, &ts_start, &ts_delta);
		delta_us = TIME_IN_US(ts_delta);
		ALOGD("KPI fingerprintsensor capture: %d ms",delta_us / 1000);
	}

	return retval;
}

int32_t dfs747_calibrate(fingerprint_data_t* device,const char *store_path)
{
	fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) device->tac_handle;

	int32_t status;
    int mode_old;
  //for new version code
  fps_init_sensor(fps_handle);
  //init_sensor(device->sysfs_fd);
  //end

	ALOGD("store_path %s",store_path);
  //set calibrate_image frist for 747B
  status = calibrate_image(device->sysfs_fd,&tac_handle->dfs_cal);
  if(status)
      ALOGE("calibrate_image fail.");

	status = calibrate_detect(device->sysfs_fd,&tac_handle->dfs_cal);
    if(status)
        ALOGE("calibrate_detect fail.");



    write_calibration_data(store_path,(unsigned char *)&tac_handle->dfs_cal,sizeof(tac_handle->dfs_cal));
    ALOGD("detect cds_offset:0x%x  thrshld:0x%x",tac_handle->dfs_cal.detect_cds_offset,tac_handle->dfs_cal.detect_thrshld);
    ALOGD("img cds_offset:0x%x  thrshld:0x%x",tac_handle->dfs_cal.img_cds_offset,tac_handle->dfs_cal.img_thrshld);

	return status;
}

int32_t dfs747_sleep(fingerprint_data_t* device,int sleep)
{
	fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) device->tac_handle;
	int retval = FPSENSOR_ERROR_OK;
    uint8_t  addr[9];
    uint8_t  data[9];
    double   sleep_us      = 0.0;
    int            mode_old   = 0;
	if((tac_handle->enrolling == true) || (tac_handle->verifying == true))
		sleep = 0;
    ALOGD("do %s enroll %d veriry %d",sleep?"sleep":"wakeup",tac_handle->enrolling,tac_handle->verifying);


        sleep_us  = (double) (DFS747_DETECT_WIDTH * DFS747_DETECT_HEIGHT * (1 + tac_handle->dfs_cal.suspend_interval) * 10 * 8 * 2500) / 1000;
        if (sleep_us < 20.0 * MSEC) {
            sleep_us = 20.0 * MSEC;
        }

        ALOGD("%s(): Suspend Interval = %.3fus\n", __func__, sleep_us);

        // Switch to detect mode
        //retval = fps_switch_mode(device->sysfs_fd, DFS747_DETECT_MODE, &mode_old);
        retval = fps_switch_mode(device->sysfs_fd, DFS747_POWER_DOWN_MODE, &mode_old);
        retval = fps_switch_mode(device->sysfs_fd, DFS747_DETECT_MODE, NULL);
		    // Disable interrupt and clear pending events
	    addr[0] = DFS747_REG_INT_CTL;
	    addr[1] = DFS747_REG_INT_EVENT;
	    data[0] = 0x00;
	    data[1] = 0x00;

	    retval = fps_multiple_write(device->sysfs_fd, addr, data, 2);
	    if (retval < 0) {
	        return -1;
	    }


    // Turn on Detect interrupt
    retval = fps_single_write(device->sysfs_fd, DFS747_REG_INT_CTL, DFS747_DETECT_EVENT);
    if (retval < 0) {
        return -1;
    }

    return 0;
}

int32_t dfs747_detect_finger(fingerprint_data_t* device)
{
	fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) device->tac_handle;
	int retval = FPSENSOR_ERROR_OK;
	static int again_count = 0;
    struct timeval tv;
    static struct timeval finger_down_tv = {0,0};
    static int finger = 0;
    static int reported = 0;
    struct timeval ts_delta;
    int delta_us;
    struct dfs747_ioc_transfer tr;
    int8_t tmp  = 0;
	  int i;

    // Turn off Detect interrupt
    /*
    retval = fps_single_write(device->sysfs_fd, DFS747_REG_INT_CTL, 0x00);
    if (retval < 0) {
        return -1;
    }
    */

	for(i=0; i < 1; i++)
	{
        ALOGD("det_detect_th = %d det_cds_offset = %d \n",det_detect_th,det_cds_offset);
        tmp = fps_scan_detect_event(device->sysfs_fd,det_detect_th,det_cds_offset,det_sleep_us,1,1);
        ALOGD("retval = %d\n",tmp);
        //retval = fps_single_read(device->sysfs_fd, DFS747_REG_INT_EVENT, &tmp);

        if (tmp < 0) {
			      retval = FINGER_DETECT_ERROR;
            ALOGD("finger error");
            break;
        }

	      retval = FINGER_DETECT_AGAIN;
		if(tmp > 0)
		{
			fps_single_write(device->sysfs_fd, DFS747_REG_INT_EVENT, 0x00);
			retval = FINGER_DETECT_PRESENT;
            if(finger == 0)
            {
                finger = 1;
                gettimeofday(&finger_down_tv,NULL);
                ALOGD("finger present");
            }
			break;
		}

  }
	if(retval == FINGER_DETECT_AGAIN)
	{
		again_count++;
		if(again_count > 2) {
            ALOGD("finger lost");
		    again_count = 0;
			retval = FINGER_DETECT_LOST;
	    }
    }	else again_count = 0;

    if(finger == 2)
    {
		if(retval == FINGER_DETECT_LOST)
        {
           if(reported)
           {
               reported = 0;
                tr.len    = 0;
                tr.opcode = DFS747_IOC_SENDKEY;

                tmp = ioctl(device->sysfs_fd, DFS747_IOC_MESSAGE(1), &tr);
                if (tmp < 0) {
                    ALOGD("%s(): Calling ioctl() failed! status = %0d\n", __func__, tmp);
                }
                else
                    ALOGD("%s(): report keyevent %d !\n", __func__, tr.len);
           }
           finger = 0;//finger up
        }
        else if(reported == 0)
        {
            gettimeofday(&tv, NULL);
            timersub(&tv, &finger_down_tv, &ts_delta);
            delta_us = TIME_IN_US(ts_delta);
            if(delta_us > 200000) {
               {
                    reported = 1;
                    tr.len    = 1;
                    tr.opcode = DFS747_IOC_SENDKEY;

                    tmp = ioctl(device->sysfs_fd, DFS747_IOC_MESSAGE(1), &tr);
                    if (tmp < 0) {
                        ALOGD("%s(): Calling ioctl() failed! status = %0d\n", __func__, tmp);
                    }
                    else
                        ALOGD("%s(): report keyevent %d !\n", __func__, tr.len);
               }
            }
        }
    }
    else if(finger == 1)
        finger = 2;//finger down

    return retval;
}



int fpsensor_load_user_db(fingerprint_data_t* device, const char* path, uint32_t path_len)
{
  int retval;
  uint8_t * pTemplate_feature = NULL;
    int i;
  char *tmpl = NULL;
   uint32_t rlen;
   bool avail = false;
    uint8_t *buf = NULL;
    FILE* fp = NULL;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) device->tac_handle;
  ALOGD("%s begin", __func__);

   strncpy(tac_handle->user_tpl_storage_path, path, path_len);
   tac_handle->path_len = path_len;

    retval = read_calibration_data(tac_handle->user_tpl_storage_path,(unsigned char *)&tac_handle->dfs_cal,sizeof(tac_handle->dfs_cal));
	if(retval)	{
		ALOGD("read %s fail,do calibration",tac_handle->user_tpl_storage_path);
    retval = calibrate_image(device->sysfs_fd,&tac_handle->dfs_cal);
		retval = calibrate_detect(device->sysfs_fd,&tac_handle->dfs_cal);

    write_calibration_data(tac_handle->user_tpl_storage_path,(unsigned char *)&tac_handle->dfs_cal,sizeof(tac_handle->dfs_cal));
	}
	else
	{
		ALOGD("read %s success,do setup calibration",tac_handle->user_tpl_storage_path);
		retval = setup_calibrate_regs(device->sysfs_fd,tac_handle->dfs_cal);
    retval = calibrate_image(device->sysfs_fd,&tac_handle->dfs_cal);
    retval = calibrate_detect(device->sysfs_fd,&tac_handle->dfs_cal);
	}
    ALOGD("detect cds_offset:0x%x  thrshld:0x%x",tac_handle->dfs_cal.detect_cds_offset,tac_handle->dfs_cal.detect_thrshld);
    ALOGD("img cds_offset:0x%x  thrshld:0x%x",tac_handle->dfs_cal.img_cds_offset,tac_handle->dfs_cal.img_thrshld);
    dump_register(device->sysfs_fd);

    retval = read_template_info(tac_handle->user_tpl_storage_path,(unsigned char *)tac_handle->tmpl_info,
            sizeof(tac_handle->tmpl_info));
	if(retval)	{
		ALOGD("read %s fail,create zero file",tac_handle->user_tpl_storage_path);
        write_template_info(tac_handle->user_tpl_storage_path,(unsigned char *)tac_handle->tmpl_info,sizeof(tac_handle->tmpl_info));
	}
	else
	{
		ALOGD("read %s success",tac_handle->user_tpl_storage_path);
	}

  tmpl = (char *)malloc(MAX_PATH_LEN);
  for(i=0;i<MAX_NBR_TEMPLATES;i++) {
    retval = snprintf(tmpl,MAX_PATH_LEN,"%stmpl%d.bin",tac_handle->user_tpl_storage_path,i + 1);
    tmpl[retval]='\0';
    if(avail == false)
    {
        pTemplate_feature = (uint8_t *)malloc(MAX_DBS_FEATURE);
        avail = true;
    }
    ALOGD("template:%s pTemplate_feature:%d",tmpl,pTemplate_feature);
    if(pTemplate_feature) {
        fp = fopen(tmpl, "r");

        if (fp == NULL) {
            ALOGE("open %s Failed,not exist?",tmpl);
            continue;
        }

        rlen = fread(pTemplate_feature, 1, MAX_DBS_FEATURE, fp);
        fclose(fp);
        if (rlen < MAX_DBS_FEATURE) {
            ALOGE("read %s error,expected %d but only read %d",tmpl,MAX_DBS_FEATURE,rlen);
            continue;
        }

        ALOGD("read template:%s success.",tmpl);
        tac_handle->pFeature_data[i] = pTemplate_feature;
        tac_handle->template_state[i] = 0xaa;
        avail = false;
    }
  }
  free(tmpl);

  if(avail)
      free(pTemplate_feature);

   get_image(device,tac_handle->pRaw_imgbuf);
   fp_finger_detection(tac_handle, &tac_handle->empty_avgValue, &tac_handle->empty_scoreValue);
  ALOGD("%s end", __func__);
  return 0;
}

int fpsensor_store_template_db(void *pHandle)
{
  int retval;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;
  char * tmpl = NULL;
  FILE * fp = NULL;

  ALOGD("%s begin", __func__);

  if((tac_handle->enroll_fid < 0) || (tac_handle->enroll_fid >= MAX_NBR_TEMPLATES)) {
     ALOGE("enroll_fid %d is out_of_range",tac_handle->enroll_fid);
     return FPSENSOR_ERROR_GENERAL;
  }
  tmpl = (char *)malloc(MAX_PATH_LEN);
  retval = snprintf(tmpl,MAX_PATH_LEN,"%stmpl%d.bin",tac_handle->user_tpl_storage_path,tac_handle->enroll_fid + 1);
  tmpl[retval]='\0';
  ALOGD("save template path %s",tmpl);
  ALOGD("store:get templateID:%d",tac_handle->pFeature_data[tac_handle->enroll_fid]);

  fp = fopen(tmpl, "w+");  // 写并覆盖
  free(tmpl);
  if (fp == NULL) {
      return -FPSENSOR_ERROR_GENERAL;
  }

  retval = fwrite(tac_handle->pFeature_data[tac_handle->enroll_fid], 1, MAX_DBS_FEATURE, fp);
  ALOGD("%s after fwrite %d", __func__,retval);
  fclose(fp);
  if (retval != MAX_DBS_FEATURE) {
      ALOGE("%s fwrite size %d is less then expected 1MB", __func__,retval);
      return -FPSENSOR_ERROR_STATE;
  }

  if(tac_handle->enrolling) {
      tac_handle->last_match_ID = tac_handle->enroll_fid;
      tac_handle->auth_id = get_64bit_rand();
      tac_handle->template_state[tac_handle->enroll_fid] = 0xaa;
      tac_handle->tmpl_info[tac_handle->enroll_fid].magic_num = TMPL_INFO_MAGIC;
      tac_handle->tmpl_info[tac_handle->enroll_fid].ID = tac_handle->enroll_fid;
      tac_handle->tmpl_info[tac_handle->enroll_fid].state = 0xaa;
      tac_handle->tmpl_info[tac_handle->enroll_fid].count = 0;
      write_template_info(tac_handle->user_tpl_storage_path,(unsigned char *)tac_handle->tmpl_info,sizeof(tac_handle->tmpl_info));
  }
  ALOGD("%s end %d", __func__,retval);

  return FPSENSOR_ERROR_OK;
}

int fpsensor_delete_template(void *pHandle, uint32_t fid)
{
  int status = FPSENSOR_ERROR_OK;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;
  char * tmpl = NULL;
  ALOGD("%s begin", __func__);


  if(tac_handle->template_state[fid - 1] == 0xaa)
    {
        tmpl = (char *)malloc(MAX_PATH_LEN);
        status = snprintf(tmpl,MAX_PATH_LEN,"%stmpl%d.bin",tac_handle->user_tpl_storage_path,fid);
        tmpl[status]='\0';
        unlink(tmpl);
        free(tmpl);
        tac_handle->template_state[fid - 1] = 0x55;
        tac_handle->tmpl_info[fid - 1].state = 0x55;
        write_template_info(tac_handle->user_tpl_storage_path,(unsigned char *)tac_handle->tmpl_info,sizeof(tac_handle->tmpl_info));
    }

  ALOGD("%s end", __func__);

  return 0;
}

int32_t dfs747_device_init(fingerprint_data_t *device)
{
    fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t *)malloc(sizeof(*tac_handle));
    memset((char *)tac_handle,0,sizeof(*tac_handle));
    tac_handle->pImg_feature_data = (uint8_t *)malloc(MAX_ONE_FEATURE);
    tac_handle->pRaw_imgbuf = (uint8_t *)malloc(DFS747_SENSOR_ROWS * DFS747_SENSOR_COLS);
  	tac_handle->width = DFS747_SENSOR_COLS;
  	tac_handle->height = DFS747_SENSOR_ROWS;
    tac_handle->last_match_ID = -1;
    device->tac_handle = tac_handle;
	device->sensor_init=dfs747_sensor_init;
	device->sensor_deinit =dfs747_sensor_deinit;
	device->sleep = dfs747_sleep;
	device->check_finger_present = dfs747_detect_finger;

	device->capture_image = dfs747_captureImage;
	device->calibrate = dfs747_calibrate;
	device->begin_enroll =dfs747_begin_enrol;
	device->enroll = dfs747_enrol;
	device->end_enroll = dfs747_end_enrol;

	device->verify = dfs747_identify;

	device->load_user_db=fpsensor_load_user_db;
	device->store_template_db=fpsensor_store_template_db;
	device->delete_template=fpsensor_delete_template;
	return 0;
}
