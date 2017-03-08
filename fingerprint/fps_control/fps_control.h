#ifndef __fps_control_h__
#define __fps_control_h__


#include <stdint.h>
#include "fps.h"
#include "fps_calibration.h"

#ifdef __cplusplus
extern "C" {
#endif


////////////////////////////////////////////////////////////////////////////////
//
// Sensor Device Handle
//

struct __fps_handle {
    int          fd;
    int          chip_id;
    int          sensor_width;
    int          sensor_height;
    int          latency;
    int          power_config;
    int          img_cal_method;
    fps_calcb_t  img_cal_callback;
    int          det_cal_method;
    fps_calcb_t  det_cal_callback;
    int          scan_det_method;
    uint8_t     *bkgnd_img;
    double       bkgnd_avg;
};


////////////////////////////////////////////////////////////////////////////////
//
// Sensor Open/Close/Reset/Init
//

// NOTE: Platform specific
fps_handle_t* fps_open_sensor(char *file);

// NOTE: Platform specific
int fps_close_sensor(fps_handle_t **handle);

// NOTE: Platform specific
int fps_reset_sensor(fps_handle_t *handle,
                     int           state);

int fps_init_sensor(fps_handle_t *handle);


////////////////////////////////////////////////////////////////////////////////
//
// Chip ID
//

enum {
    F747A_CHIP_ID = 0x1100,
    F747B_CHIP_ID = 0x0747,
    F767A_CHIP_ID = 0x0767,
};

// NOTE: Platform specific
int fps_get_chip_id(fps_handle_t *handle);


////////////////////////////////////////////////////////////////////////////////
//
// Sensor Dimension
//

int fps_get_sensor_width(fps_handle_t *handle);

int fps_get_sensor_height(fps_handle_t *handle);

int fps_get_sensor_size(fps_handle_t *handle);


////////////////////////////////////////////////////////////////////////////////
//
// Power Configuration
//

enum {
    FPS_POWER_CONFIG_1V8_3V3  = 0,
    FPS_POWER_CONFIG_1V8_2V8  = 1,
    FPS_POWER_CONFIG_1V8_ONLY = 2,
    FPS_POWER_CONFIG_EXTERNAL = 3,
};

int fps_set_power_config(fps_handle_t *handle,
                         int           config);

int fps_get_power_config(fps_handle_t *handle,
                         int          *config);


////////////////////////////////////////////////////////////////////////////////
//
// Register Access
//

// NOTE: Platform specific
int fps_multiple_read(fps_handle_t *handle,
                      uint8_t      *addr,
                      uint8_t      *data,
                      size_t        length);

// NOTE: Platform specific
int fps_multiple_write(fps_handle_t *handle,
                       uint8_t      *addr,
                       uint8_t      *data,
                       size_t        length);

int fps_single_read(fps_handle_t *handle,
                    uint8_t       addr,
                    uint8_t      *data);

int fps_single_write(fps_handle_t *handle,
                     uint8_t       addr,
                     uint8_t       data);

int fps_set_bits(fps_handle_t *handle,
                 uint8_t       addr,
                 uint8_t       bits);

int fps_clear_bits(fps_handle_t *handle,
                   uint8_t       addr,
                   uint8_t       bits);

int fps_check_bits(fps_handle_t *handle,
                   uint8_t       addr,
                   int           bits);


////////////////////////////////////////////////////////////////////////////////
//
// Interrupt Operations
//

int fps_enable_interrupt(fps_handle_t *handle,
                         int           event);

int fps_disable_interrupt(fps_handle_t *handle,
                          int           event);

int fps_check_interrupt(fps_handle_t *handle,
                        int           event);

int fps_clear_interrupt(fps_handle_t *handle,
                        int           event);

#define FPS_DISABLE_AND_CLEAR_INTERRUPT(_handle_, _event_) \
    do { \
        (void) fps_disable_interrupt((_handle_), (_event_)); \
        (void) fps_clear_interrupt((_handle_), (_event_)); \
    } while (0)


////////////////////////////////////////////////////////////////////////////////
//
// Sensor Parameter Settings
//

enum {
    FPS_PARAM_CDS_OFFSET_0     = (0 << 8),
    FPS_PARAM_CDS_OFFSET_1     = (1 << 8),
    FPS_PARAM_PGA_GAIN_0       = (2 << 8),
    FPS_PARAM_PGA_GAIN_1       = (3 << 8),
    FPS_PARAM_DETECT_THRESHOLD = (4 << 8),
};

int fps_set_sensor_parameter(fps_handle_t *handle,
                             int           flag,
                             int           value);

int fps_get_sensor_parameter(fps_handle_t *handle,
                             int           flag,
                             int          *value);

int fps_set_sensing_area(fps_handle_t *handle,
                         int           mode,
                         int           col_begin,
                         int           col_end,
                         int           row_begin,
                         int           row_end);

int fps_get_sensing_area(fps_handle_t *handle,
                         int           mode,
                         int          *col_begin,
                         int          *col_end,
                         int          *row_begin,
                         int          *row_end);

int fps_set_suspend_frames(fps_handle_t *handle,
                           int           frames);

int fps_get_suspend_frames(fps_handle_t *handle,
                           int          *frames);

double fps_calculate_suspend_time(int det_size,
                                  int frames);


////////////////////////////////////////////////////////////////////////////////
//
// Mode Switch
//

int fps_switch_mode(fps_handle_t *handle,
                    int           mode_new,
                    int          *mode_old);


////////////////////////////////////////////////////////////////////////////////
//
// Image Mode Operations
//

// NOTE: Platform specific
int fps_get_raw_image(fps_handle_t *handle,
                      int           img_width,
                      int           img_height,
                      uint8_t      *img_buf);

int fps_set_background_image(fps_handle_t *handle,
                             int           img_width,
                             int           img_height,
                             uint8_t      *img_buf);

int fps_get_background_image(fps_handle_t *handle,
                             int           img_width,
                             int           img_height,
                             uint8_t      *img_buf);

int fps_get_background_average(fps_handle_t *handle,
                               double       *img_avg);

int fps_get_averaged_image(fps_handle_t *handle,
                           int           img_width,
                           int           img_height,
                           int           frms_to_avg,
                           uint8_t      *img_buf,
                           double       *img_avg,
                           double       *img_var,
                           double       *img_noise);

int fps_get_finger_image(fps_handle_t *handle,
                         int           img_width,
                         int           img_height,
                         int           frms_to_avg,
                         uint8_t      *img_buf,
                         double       *img_dr,
                         double       *img_var,
                         double       *img_noise);


////////////////////////////////////////////////////////////////////////////////
//
// Detect Mode Operations
//

int fps_set_scan_detect_method(fps_handle_t *handle,
                               int           method);

int fps_get_scan_detect_method(fps_handle_t *handle,
                               int          *method);

int fps_scan_detect_event(fps_handle_t *handle,
                          double        sleep_us);


////////////////////////////////////////////////////////////////////////////////
//
// Helpers
//

// NOTE: Platform specific
int fps_wait_event(fps_handle_t *handle,
                   double        sleep_us);

// NOTE: Platform specific
void fps_sleep(double sleep_us);


#ifdef __cplusplus
}
#endif


#endif // __fps_control_h__
