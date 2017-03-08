#include <stdint.h>
#include <stdlib.h>
#include "fps_register.h"
#include "fps_control.h"
#include "fps_calibration.h"
#include "debug.h"


////////////////////////////////////////////////////////////////////////////////
//
// Image Caliration Method 1
// -----------------------------------------------------------------------------
// NOTE:
//


////////////////////////////////////////////////////////////////////////////////
//
// Image Caliration Method 2
// -----------------------------------------------------------------------------
// NOTE:
//

static int
fps_image_calibration_2(fps_handle_t *handle,
                        int           col_begin,
                        int           col_end,
                        int           row_begin,
                        int           row_end,
                        int           frms_to_avg)
{
    // Pre-defined parameters
    const int    upper_bound = 240;
    const int    lower_bound = 10;
    const double ratio       = 0.05;
    const int    cds_upper   = FPS_MAX_CDS_OFFSET_1 - 0x80;
    const int    cds_lower   = FPS_MIN_CDS_OFFSET_1 + 0x00;
    const int    cds_step    = 1;
    const int    pga_upper   = FPS_MAX_PGA_GAIN_1   - 0x02;
    const int    pga_lower   = FPS_MIN_PGA_GAIN_1   + 0x05;
    const int    pga_step    = 1;

    int           status = 0;
    int           img_width;
    int           img_height;
    int           scan_col_begin;
    int           scan_col_end;
    int           scan_row_begin;
    int           scan_row_end;
    int           scan_width;
    int           scan_height;
    size_t        scan_size;
    uint8_t       addr[3];
    uint8_t       data[3];
    int           curr_cds;
    int           curr_pga;
    double        img_var;
    double        img_noise;
    int           brighter_pixels;
    int           darker_pixels;
    int           too_bright;
    int           too_dark;
    fps_calinfo_t info;
    int           r;
    int           c;

    img_width  = col_end - col_begin + 1;
    img_height = row_end - row_begin + 1;

    scan_col_begin = col_begin + (img_width  / 4);
    scan_col_end   = col_end   - (img_width  / 4);
    scan_row_begin = row_begin + (img_height / 4);
    scan_row_end   = row_end   - (img_height / 4);

    scan_width  = scan_col_end - scan_col_begin + 1;
    scan_height = scan_row_end - scan_row_begin + 1;
    scan_size   = scan_width * scan_height;

    if ((scan_width < 0) || (scan_height < 0)) {
        return -1;
    }

    // Disable and clear all interrupts
    FPS_DISABLE_AND_CLEAR_INTERRUPT(handle, FPS_ALL_EVENTS);

    // Set image window
    status = fps_set_sensing_area(handle, FPS_IMAGE_MODE,
                                  col_begin, col_end,
                                  row_begin, row_end);
    if (status < 0) {
        return status;
    }

    // Initial conditions
    addr[0] = FPS_REG_IMG_CDS_CTL_0;
    addr[1] = FPS_REG_IMG_CDS_CTL_1;
    addr[2] = FPS_REG_IMG_PGA1_CTL;

    status = fps_multiple_read(handle, addr, data, 3);
    if (status < 0) {
        return status;
    }

    curr_cds = cds_upper;
    curr_pga = pga_upper;

    while (1) {
        DEBUG(TEXT("CDS Offset = 0x%03X, PGA Gain = 0x%02X\n"),
              curr_cds, curr_pga);

        data[0] = ((uint8_t) ((curr_cds & 0x100) >> 1)) | (data[0] & 0x7F);
        data[1] = ((uint8_t) ((curr_cds & 0x0FF) >> 0));
        data[2] = ((uint8_t) ((curr_pga & 0x0F ) >> 0)) | (data[2] & 0xF0);

        status = fps_multiple_write(handle, addr, data, 3);
        if (status < 0) {
            return status;
        }

        // Get an image
        status = fps_get_averaged_image(handle,
                                        img_width,
                                        img_height,
                                        frms_to_avg,
                                        handle->bkgnd_img,
                                        &handle->bkgnd_avg,
                                        &img_var,
                                        &img_noise);
        if (status < 0) {
            return status;
        }

        if (handle->img_cal_callback != NULL) {
            info.img_buf      = handle->bkgnd_img;
            info.img_avg      = handle->bkgnd_avg;
            info.img_var      = img_var;
            info.img_noise    = img_noise;
            info.cds_offset_1 = curr_cds;
            info.pga_gain_1   = curr_pga;

            status = handle->img_cal_callback(&info);
            if (status < 0) {
                return status;
            }
        }

        // Do calibration
        brighter_pixels = 0;
        darker_pixels   = 0;

        for (r = scan_row_begin; r <= scan_row_end; r++) {
        for (c = scan_col_begin; c <= scan_col_end; c++) {
            if (handle->bkgnd_img[r * img_width + c] > upper_bound) {
                brighter_pixels++;
            }
            if (handle->bkgnd_img[r * img_width + c] < lower_bound) {
                darker_pixels++;
            }
        }}

        too_bright = (brighter_pixels >= (int) (((double) scan_size) * ratio));
        too_dark   = (darker_pixels   >= (int) (((double) scan_size) * ratio));

        DEBUG(TEXT("Brighter = %0d, Darker = %0d (%s)\n"),
              brighter_pixels, darker_pixels,
              (too_bright ? TEXT("Too Bright") :
               too_dark   ? TEXT("Too Dark")   : TEXT("OK")));

        if (too_bright == TRUE) {
            if (curr_cds < cds_upper) {
                curr_cds += cds_step;
                continue;
            }

            if (curr_pga > pga_lower) {
                curr_pga -= pga_step;
                curr_cds  = cds_upper;
                continue;
            }

            ERROR(TEXT("Too bright but no more settings!\n"));
            break;
        }

        if (too_dark == TRUE) {
            if (curr_cds > cds_lower) {
                curr_cds -= cds_step;
                continue;
            }

            if (curr_pga < pga_upper) {
                curr_pga += pga_step;
                curr_cds  = cds_upper;
                continue;
            }

            ERROR(TEXT("Too dark but no more settings!\n"));
            break;
        }

        break;
    }

    if ((too_bright == FALSE) && (too_dark == FALSE)) {
        return 0;
    } else {
        return -1;
    }
}


////////////////////////////////////////////////////////////////////////////////
//
// Image Calibration APIs
//

int
fps_set_image_calibration_method(fps_handle_t *handle,
                                 int           method)
{
    handle->img_cal_method = method;
    return 0;
}

int
fps_get_image_calibration_method(fps_handle_t *handle,
                                 int          *method)
{
    *method = handle->img_cal_method;
    return 0;
}

int
fps_set_image_calibration_callback(fps_handle_t *handle,
                                   fps_calcb_t   callback)
{
    handle->img_cal_callback = callback;
    return 0;
}

int
fps_image_calibration(fps_handle_t *handle,
                      int           img_width,
                      int           img_height,
                      int           frms_to_avg)
{
    int status = 0;
    int method;
    int sensor_width;
    int sensor_height;
    int col_begin;
    int col_end;
    int row_begin;
    int row_end;

    method = handle->img_cal_method;

    sensor_width  = handle->sensor_width;
    sensor_height = handle->sensor_height;

    if (method == 2) {
        col_begin = (sensor_width - img_width) / 2;
        col_end   = col_begin + img_width - 1;
        row_begin = (sensor_height - img_height) / 2;
        row_end   = row_begin + img_height - 1;

        status = fps_image_calibration_2(handle,
                                         col_begin,
                                         col_end,
                                         row_begin,
                                         row_end,
                                         frms_to_avg);
    } else {
        status = -1;
    }

    return status;
}
