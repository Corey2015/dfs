#ifndef __fps_h__
#define __fps_h__


#include <stdint.h>


#if defined(__cplusplus)
extern "C" {
#endif


////////////////////////////////////////////////////////////////////////////////
//
// Sensor Device Handle
//

typedef struct __fps_handle fps_handle_t;


////////////////////////////////////////////////////////////////////////////////
//
// Sensor Open/Close/Reset/Init
//

extern fps_handle_t *fps_open_sensor(char *file);

extern int fps_close_sensor(fps_handle_t **handle);
                                 
extern int fps_reset_sensor(fps_handle_t *handle,
                            int           state);
                                 
extern int fps_init_sensor(fps_handle_t *handle);


////////////////////////////////////////////////////////////////////////////////
//
// Mode Switch
//

enum {
    FPS_IMAGE_MODE      = 0,
    FPS_DETECT_MODE     = 1,
    FPS_POWER_DOWN_MODE = 2,
};

extern int fps_switch_mode(fps_handle_t *handle,
                           int           mode_new,
                           int          *mode_old);


////////////////////////////////////////////////////////////////////////////////
//
// Image Mode Operations
//

extern int fps_get_averaged_image(fps_handle_t *handle,
                                  int           img_width,
                                  int           img_height,
                                  int           frms_to_avg,
                                  uint8_t      *img_buf,
                                  double       *img_avg,
                                  double       *img_var,
                                  double       *img_noise);

extern int fps_get_finger_image(fps_handle_t *handle,
                                int           img_width,
                                int           img_height,
                                int           frms_to_avg,
                                uint8_t      *img_buf,
                                double       *img_dr,
                                double       *img_var,
                                double       *img_noise);

extern int fps_image_calibration(fps_handle_t *handle,
                                 int           img_width,
                                 int           img_height,
                                 int           frms_to_avg);


////////////////////////////////////////////////////////////////////////////////
//
// Detect Mode Operations
//

extern int fps_scan_detect_event(fps_handle_t *handle,
                                 double        sleep_us);

extern int fps_detect_calibration(fps_handle_t *handle,
                                  int           det_width,
                                  int           det_height,
                                  int           frms_to_susp);


#if defined(__cplusplus)
}
#endif


#endif // __fps_h__
