#include <stdint.h>
#include <stdlib.h>
#include "fps_register.h"
#include "fps_control.h"
#include "fps_calibration.h"
#include "debug.h"


////////////////////////////////////////////////////////////////////////////////
//
// Detect Calibration Helpers
//

static int
fps_search_detect_window(uint8_t *bkgnd_img,
                         int      img_width,
                         int      img_height,
                         int      det_width,
                         int      det_height,
                         int      scan_width,
                         int      scan_height,
                         int     *det_col_begin,
                         int     *det_col_end,
                         int     *det_row_begin,
                         int     *det_row_end,
                         double  *det_avg,
                         double  *det_var)
{
    // Pre-defined parameters
    const double var_upper = 50.0;

    int     status = 0;
    int     det_size;
    int     ver_scan_cnt;
    int     hor_scan_cnt;
    int     total_scan_cnt;
    double *all_avg;
    double *all_var;
    int     col_start;
    int     row_start;
    double  pix_sum;
    double  sqr_sum;
    double  pix;
    double  avg;
    double  var;
    int     w;
    int     sc;
    int     sr;
    int     c;
    int     r;

    det_size = det_width * det_height;

    hor_scan_cnt   = scan_width  - det_width  + 1;
    ver_scan_cnt   = scan_height - det_height + 1;
    total_scan_cnt = hor_scan_cnt * ver_scan_cnt;

    all_avg = NULL;
    all_var = NULL;

    // Create to record all detect windows' average and variance
    all_avg = (double *) malloc(sizeof(double) * total_scan_cnt);
    if (all_avg == NULL) {
        status = -1;
        goto fps_search_detect_window_end;
    }

    all_var = (double *) malloc(sizeof(double) * total_scan_cnt);
    if (all_var == NULL) {
        status = -1;
        goto fps_search_detect_window_end;
    }

    for (w = 0; w < total_scan_cnt; w++) {
        all_avg[w] = 0.0;
        all_var[w] = 0.0;
    }

    col_start = (img_width  - scan_width ) / 2;
    row_start = (img_height - scan_height) / 2;

    // Walk through all candidate windows vertically
    for (sr = row_start; sr < (row_start + ver_scan_cnt); sr++) {
    // Walk through all candidate windows horizontally
    for (sc = col_start; sc < (col_start + hor_scan_cnt); sc++) {

        // Calculate average and variance of candidate window
        pix_sum = 0;
        sqr_sum = 0;
        for (r = sr; r < (sr + det_height); r++) {
        for (c = sc; c < (sc + det_width ); c++) {
            pix      = (double) bkgnd_img[r * img_width + c];
            pix_sum +=  pix;
            sqr_sum += (pix * pix);
        }}

        avg =  pix_sum / det_size;
        var = (sqr_sum / det_size) - (avg * avg);

        w = (sr - row_start) * hor_scan_cnt + (sc - col_start);
        if (var > all_var[w]) {
            all_avg[w] = avg;
            all_var[w] = var;
        }

        DEBUG(TEXT("Window (CB,RB):(CE,RE) = (%0d,%0d):(%0d,%0d) Avg. = %0.3f, Var. = %0.3f\n"),
              sc, sr, (sc + det_width - 1), (sr + det_height - 1), all_avg[w], all_var[w]);
    }}

    // Search the window with minimum variance
    *det_col_begin = col_start;
    *det_col_end   = col_start + det_width  - 1;
    *det_row_begin = row_start;
    *det_row_end   = row_start + det_height - 1;;
    var            = all_var[0];

    for (sr = row_start; sr < (row_start + ver_scan_cnt); sr++) {
    for (sc = col_start; sc < (col_start + hor_scan_cnt); sc++) {

        w = (sr - row_start) * hor_scan_cnt + (sc - col_start);

        if ((all_var[w] < var) && (var != 0)) {
            *det_col_begin = sc;
            *det_col_end   = sc + det_width  - 1;
            *det_row_begin = sr;
            *det_row_end   = sr + det_height - 1;

            avg = all_avg[w];
            var = all_var[w];
        }
    }}

    if (var >= var_upper) {
        DEBUG(TEXT("Too bad to search a good detect window!\n"));
        status = -1;
        goto fps_search_detect_window_end;
    }

    if (det_avg != NULL) {
        *det_avg = avg;
    }

    if (det_var != NULL) {
        *det_var = var;
    }

    DEBUG(TEXT("Window (CB,RB):(CE,RE) = (%0d,%0d):(%0d,%0d) Avg. = %0.3f, Var. = %0.3f\n"),
          *det_col_begin, *det_row_begin, *det_col_end, *det_row_end, avg, var);

fps_search_detect_window_end:

    if (all_avg != NULL) {
        free(all_avg);
    }

    if (all_var != NULL) {
        free(all_var);
    }

    return status;
}

static int
fps_search_detect_threshold(fps_handle_t *handle,
                            int           upper_bound,
                            int           lower_bound,
                            double        sleep_us,
                            int           extra_steps,
                            int          *detect_th)
{
    // Pre-defined parameters
    const int scan_limit = 8;

    int           status = 0;
    int           curr_cds;
    uint8_t       data;
    int           det_upper;
    int           det_middle;
    int           det_lower;
    int           scan_cnt;
    fps_calinfo_t info;

    if (handle->det_cal_callback != NULL) {
        status = fps_get_sensor_parameter(handle,
                                          (FPS_DETECT_MODE | FPS_PARAM_CDS_OFFSET_1),
                                          &curr_cds);
        if (status < 0) {
            return status;
        }
    }

    status = fps_single_read(handle, FPS_REG_V_DET_SEL, &data);
    if (status < 0) {
        return status;
    }

    det_upper  = upper_bound;
    det_lower  = lower_bound;
    det_middle = (upper_bound + lower_bound) / 2;

    while ((det_upper - det_lower) > 2) {
        DEBUG(TEXT("Detect Threshold range = 0x%02X : 0x%02X : 0x%02X\n"),
              det_upper, det_middle, det_lower);

        data = ((uint8_t) ((det_middle & 0x3F) >> 0)) | (data & 0xC0);

        status = fps_single_write(handle, FPS_REG_V_DET_SEL, data);
        if (status < 0) {
            return status;
        }

        for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
            status = fps_scan_detect_event(handle, sleep_us);
            if (status < 0) {
                return status;
            }

            if ((status > 0) && (scan_cnt > 0)) {
                break;
            }
        }

        if (handle->det_cal_callback != NULL) {
            info.cds_offset_1 = curr_cds;
            info.detect_th    = det_middle;

            status = handle->det_cal_callback(&info);
            if (status < 0) {
                return status;
            }
        }

        if (scan_cnt == scan_limit) {
            det_upper = det_middle;
        } else {
            det_lower = det_middle;
        }

        det_middle = (det_upper + det_lower) / 2;
    }

    if (det_middle == upper_bound) {
        ERROR(TEXT("Maximum Detect Threshold reached!\n"));
        return -2;
    }
    if (det_middle == lower_bound) {
        ERROR(TEXT("Minimum Detect Threshold reached!\n"));
        return -2;
    }

    *detect_th = det_middle + extra_steps;
    if (*detect_th > FPS_MAX_DETECT_TH) {
        *detect_th = FPS_MAX_DETECT_TH;
    }
    if (*detect_th < FPS_MIN_DETECT_TH) {
        *detect_th = FPS_MIN_DETECT_TH;
    }

    status = fps_single_write(handle, FPS_REG_V_DET_SEL, *detect_th);

    return status;
}

static int
fps_search_detect_cds_offset(fps_handle_t *handle,
                             int           upper_bound,
                             int           lower_bound,
                             double        sleep_us,
                             int           extra_steps,
                             int          *cds_offset)
{
    // Pre-defined parameters
    const int scan_limit = 8;

    int           status = 0;
    int           curr_det;
    uint8_t       addr[2];
    uint8_t       data[2];
    int           cds_upper;
    int           cds_middle;
    int           cds_lower;
    int           scan_cnt;
    fps_calinfo_t info;

    if (handle->det_cal_callback != NULL) {
        status = fps_get_sensor_parameter(handle, FPS_PARAM_DETECT_THRESHOLD, &curr_det);
        if (status < 0) {
            return status;
        }
    }

    addr[0] = FPS_REG_DET_CDS_CTL_0;
    addr[1] = FPS_REG_DET_CDS_CTL_1;
    status = fps_multiple_read(handle, addr, data, 2);
    if (status < 0) {
        return status;
    }

    cds_upper  = upper_bound;
    cds_lower  = lower_bound;
    cds_middle = (upper_bound + lower_bound) / 2;

    while ((cds_upper - cds_lower) > 2) {
        DEBUG(TEXT("CDS Offset range = 0x%03X : 0x%03X : 0x%03X\n"),
              cds_upper, cds_middle, cds_lower);

        data[0] = ((uint8_t) ((cds_middle & 0x100) >> 1)) | (data[0] & 0x7F);
        data[1] = ((uint8_t) ((cds_middle & 0x0FF) >> 0));
        status = fps_multiple_write(handle, addr, data, 2);
        if (status < 0) {
            return status;
        }

        for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
            status = fps_scan_detect_event(handle, sleep_us);
            if (status < 0) {
                return status;
            }

            if ((status == 0) && (scan_cnt > 0)) {
                break;
            }
        }

        if (handle->det_cal_callback != NULL) {
            info.cds_offset_1 = cds_middle;
            info.detect_th    = curr_det;

            status = handle->det_cal_callback(&info);
            if (status < 0) {
                return status;
            }
        }

        if (scan_cnt == scan_limit) {
            cds_lower = cds_middle;
        } else {
            cds_upper = cds_middle;
        }

        cds_middle = (cds_upper + cds_lower) / 2;
    }

    if (cds_middle == upper_bound) {
        ERROR(TEXT("Maximum CDS Offset reached!\n"));
        return -2;
    }
    if (cds_middle == lower_bound) {
        ERROR(TEXT("Minimum CDS Offset reached!\n"));
        return -2;
    }

    *cds_offset = cds_middle + extra_steps;
    if (*cds_offset > FPS_MAX_CDS_OFFSET_1) {
        *cds_offset = FPS_MAX_CDS_OFFSET_1;
    }
    if (*cds_offset < FPS_MIN_CDS_OFFSET_1) {
        *cds_offset = FPS_MIN_CDS_OFFSET_1;
    }

    data[0] = ((uint8_t) ((*cds_offset & 0x100) >> 1)) | (data[0] & 0x7F);
    data[1] = ((uint8_t) ((*cds_offset & 0x0FF) >> 0));
    status = fps_multiple_write(handle, addr, data, 2);

    return status;
}


////////////////////////////////////////////////////////////////////////////////
//
// Detect Calibration Method 1
// -----------------------------------------------------------------------------
// NOTE:
//


////////////////////////////////////////////////////////////////////////////////
//
// Detect Calibration Method 2
// -----------------------------------------------------------------------------
// NOTE:
//


////////////////////////////////////////////////////////////////////////////////
//
// Detect Calibration Method 3
// -----------------------------------------------------------------------------
// NOTE:
//


////////////////////////////////////////////////////////////////////////////////
//
// Detect Calibration Method 4
// -----------------------------------------------------------------------------
// NOTE:
//

static int
fps_detect_calibration_4(fps_handle_t *handle,
                         int           col_begin,
                         int           col_end,
                         int           row_begin,
                         int           row_end,
                         int           frms_to_susp)
{
    // Pre-defined parameters
    const int retry_limit      = 5;
    const int extra_detect_th  = 1;
    const int extra_cds_offset = 6;
    const int det_max          = FPS_MAX_DETECT_TH;
    const int det_min          = FPS_MIN_DETECT_TH;
    const int cds_max          = FPS_MAX_CDS_OFFSET_1 / 2;
    const int cds_min          = FPS_MIN_CDS_OFFSET_1;
    const int init_cds         = FPS_MAX_CDS_OFFSET_1 / 2;
    const int init_pga         = 0x03;

    int     status = 0;
    int     det_width;
    int     det_height;
    uint8_t addr[3];
    uint8_t data[3];
    int     curr_cds;
    int     curr_det;
    double  sleep_us;
    int     retry_cnt;

    det_width  = col_end - col_begin + 1;
    det_height = row_end - row_begin + 1;

    // Disable and clear all interrupts
    FPS_DISABLE_AND_CLEAR_INTERRUPT(handle, FPS_ALL_EVENTS);

    // Set detect window
    status = fps_set_sensing_area(handle, FPS_DETECT_MODE,
                                  col_begin, col_end,
                                  row_begin, row_end);
    if (status < 0) {
        return status;
    }

    // Cache the following registers to avoid 'read-modify-write' penalty
    addr[0] = FPS_REG_DET_CDS_CTL_0;
    addr[1] = FPS_REG_DET_CDS_CTL_1;
    addr[2] = FPS_REG_DET_PGA1_CTL;

    status = fps_multiple_read(handle, addr, data, 3);
    if (status < 0) {
        return status;
    }

    // Specify fixed CDS offset and PGA gain in the Detect Mode
    data[0] = ((uint8_t) ((init_cds & 0x100) >> 1)) | (data[0] & 0x7F);
    data[1] = ((uint8_t) ((init_cds & 0x0FF) >> 0));
    data[2] = ((uint8_t) (init_pga & 0x0F)) | (data[2] & 0xF0);

    status = fps_multiple_write(handle, addr, data, 3);
    if (status < 0) {
        return status;
    }

    // Set suspend interval
    status = fps_set_suspend_frames(handle, frms_to_susp);
    if (status < 0) {
        return status;
    }

    // Calculate corresponding sleep time
    sleep_us = fps_calculate_suspend_time((det_width * det_height), frms_to_susp) * 2;

    // Phase 1: Search detect threshold
    for (retry_cnt = 0; retry_cnt < retry_limit; retry_cnt++) {

        status = fps_search_detect_threshold(handle,
                                             det_max,
                                             det_min,
                                             sleep_us,
                                             extra_detect_th,
                                             &curr_det);
        if (status == -2) {
            continue;
        }
        if (status < 0) {
            return status;
        }

        break;
    }

    if (retry_cnt == retry_limit) {
        DEBUG("Retry time out! Exit...\n");
        return -1;
    }

    // Phase 2: Search CDS offset
    for (; retry_cnt < retry_limit; retry_cnt++) {

        status = fps_search_detect_cds_offset(handle,
                                              cds_max,
                                              cds_min,
                                              sleep_us,
                                              extra_cds_offset,
                                              &curr_cds);
        if (status == -2) {
            continue;
        }
        if (status < 0) {
            return status;
        }

        break;
    }

    if (retry_cnt == retry_limit) {
        DEBUG("Retry time out! Exit...\n");
        return -1;
    }

    return status;
}


////////////////////////////////////////////////////////////////////////////////
//
// Detect Calibration APIs
//

int
fps_set_detect_calibration_method(fps_handle_t *handle,
                                  int           method)
{
    handle->det_cal_method = method;
    return 0;
}

int
fps_get_detect_calibration_method(fps_handle_t *handle,
                                  int          *method)
{
    *method = handle->det_cal_method;
    return 0;
}

int
fps_set_detect_calibration_callback(fps_handle_t *handle,
                                    fps_calcb_t   callback)
{
    handle->det_cal_callback = callback;
    return 0;
}

int
fps_detect_calibration(fps_handle_t *handle,
                       int           det_width,
                       int           det_height,
                       int           frms_to_susp)
{
    int status = 0;
    int method;
    int img_col_begin;
    int img_col_end;
    int img_row_begin;
    int img_row_end;
    int img_width;
    int img_height;
    int scan_width;
    int scan_height;
    int det_col_begin;
    int det_col_end;
    int det_row_begin;
    int det_row_end;

    method = handle->det_cal_method;

    if (method == 4) {

        status = fps_get_sensing_area(handle, FPS_IMAGE_MODE,
                                      &img_col_begin, &img_col_end,
                                      &img_row_begin, &img_row_end);
        if (status < 0) {
            return status;
        }

        img_width  = img_col_end - img_col_begin + 1;
        img_height = img_row_end - img_row_begin + 1;

        scan_width  = det_width;
        scan_height = img_height - 16;

        status = fps_search_detect_window(handle->bkgnd_img,
                                          img_width,
                                          img_height,
                                          det_width,
                                          det_height,
                                          scan_width,
                                          scan_height,
                                          &det_col_begin,
                                          &det_col_end,
                                          &det_row_begin,
                                          &det_row_end,
                                          NULL, NULL);
        if (status < 0) {
            return status;
        }

        status = fps_detect_calibration_4(handle,
                                          det_col_begin,
                                          det_col_end,
                                          det_row_begin,
                                          det_row_end,
                                          frms_to_susp);
        if (status < 0) {
            return status;
        }
    } else {
        status = -1;
    }

    return status;
}
