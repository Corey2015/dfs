#ifndef __fps_calibration_h__
#define __fps_calibration_h__


#include <stdint.h>
#include "fps_control.h"

#ifdef __cplusplus
extern "C" {
#endif


////////////////////////////////////////////////////////////////////////////////
//
// Calibration Infomration
//

struct __fps_calinfo {
    uint8_t *img_buf;
    double   img_avg;
    double   img_var;
    double   img_noise;
    int      cds_offset_0;
    int      cds_offset_1;
    int      pga_gain_0;
    int      pga_gain_1;
    int      detect_th;
};

typedef struct __fps_calinfo fps_calinfo_t;

typedef int (*fps_calcb_t) (fps_calinfo_t *info);


////////////////////////////////////////////////////////////////////////////////
//
// Image Calibration
//

int fps_set_image_calibration_method(fps_handle_t *handle,
                                     int           method);

int fps_get_image_calibration_method(fps_handle_t *handle,
                                     int          *method);

int fps_set_image_calibration_callback(fps_handle_t *handle,
                                       fps_calcb_t   callback);

int fps_image_calibration(fps_handle_t *handle,
                          int           img_width,
                          int           img_height,
                          int           frms_to_avg);


////////////////////////////////////////////////////////////////////////////////
//
// Detect Calibration
//

int fps_set_detect_calibration_method(fps_handle_t *handle,
                                      int           method);

int fps_get_detect_calibration_method(fps_handle_t *handle,
                                      int          *method);

int fps_set_detect_calibration_callback(fps_handle_t *handle,
                                        fps_calcb_t   callback);

int fps_detect_calibration(fps_handle_t *handle,
                           int           det_width,
                           int           det_height,
                           int           frms_to_susp);


#ifdef __cplusplus
}
#endif


#endif // __fps_calibration_h__
