#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include "fps_register.h"
#include "fps_control.h"
#include "debug.h"


////////////////////////////////////////////////////////////////////////////////
//
// Sensor Open/Close/Reset/Init
//

int
fps_init_sensor(fps_handle_t *handle)
{
    int     status = 0;
    uint8_t addr[4];
    uint8_t data[4];

    // Disable and clear interrupts
    status = fps_disable_interrupt(handle, FPS_ALL_EVENTS);
    if (status < 0) {
        return status;
    }

    status = fps_clear_interrupt(handle, FPS_ALL_EVENTS);
    if (status < 0) {
        return status;
    }

    // Enter power down mode
    status = fps_switch_mode(handle, FPS_POWER_DOWN_MODE, NULL);
    if (status < 0) {
        return status;
    }

    // Specify the power source configuration
    addr[0] = FPS_REG_CPR_CTL_0;
    addr[1] = FPS_REG_CPR_CTL_1;

    switch (handle->power_config) {
        case FPS_POWER_CONFIG_1V8_3V3 :
            data[0] = 0x10;
            data[1] = 0x3C;
            break;

        case FPS_POWER_CONFIG_1V8_2V8 :
            data[0] = 0xFC;
            data[1] = 0x3C;
            break;

        case FPS_POWER_CONFIG_1V8_ONLY :
            data[0] = 0xFC;
            data[1] = 0x3C;
            break;

        case FPS_POWER_CONFIG_EXTERNAL :
            data[0] = 0x00;
            data[1] = 0x00;
            break;
    }

    status = fps_multiple_write(handle, addr, data, 2);
    if (status < 0) {
        return status;
    }

    fps_sleep(50.0 * 1000);

    // Settings for sensing enhancement
    addr[0] = FPS_REG_IMG_PGA0_CTL;
    addr[1] = FPS_REG_ANA_I_SET_1;
    addr[2] = FPS_REG_SET_ANA_0;
    addr[3] = FPS_REG_MISC_PWR_CTL_1;

    data[0] = 0x20;
    data[1] = 0x03; // Michael: 0x03
    data[2] = 0x1F; // Michael: 0x07
    data[3] = 0x20;

    status = fps_multiple_write(handle, addr, data, 4);
    if (status < 0) {
        return status;
    }

    // Enter image mode
    status = fps_switch_mode(handle, FPS_IMAGE_MODE, NULL);

    return status;
}


////////////////////////////////////////////////////////////////////////////////
//
// Sensor Dimension
//

int
fps_get_sensor_width(fps_handle_t *handle)
{
    return handle->sensor_width;
}

int
fps_get_sensor_height(fps_handle_t *handle)
{
    return handle->sensor_height;
}

int
fps_get_sensor_size(fps_handle_t *handle)
{
    return (handle->sensor_width * handle->sensor_height);
}


////////////////////////////////////////////////////////////////////////////////
//
// Power Configuration
//

int
fps_set_power_config(fps_handle_t *handle,
                     int           config)
{
    handle->power_config = config;
    return 0;
}

int
fps_get_power_config(fps_handle_t *handle,
                     int          *config)
{
    *config = handle->power_config;
    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// Register Access
//

int
fps_single_read(fps_handle_t *handle,
                uint8_t       addr,
                uint8_t      *data)
{
    return fps_multiple_read(handle, &addr, data, 1);
}

int
fps_single_write(fps_handle_t *handle,
                 uint8_t       addr,
                 uint8_t       data)
{
    return fps_multiple_write(handle, &addr, &data, 1);
}

int
fps_set_bits(fps_handle_t *handle,
             uint8_t       addr,
             uint8_t       bits)
{
    int     status = 0;
    uint8_t data;

    status = fps_single_read(handle, addr, &data);
    if (status < 0) {
        return status;
    }

    data |= ((uint8_t) bits);

    status = fps_single_write(handle, addr, data);

    return status;
}

int
fps_clear_bits(fps_handle_t *handle,
               uint8_t       addr,
               uint8_t       bits)
{
    int     status = 0;
    uint8_t data;

    status = fps_single_read(handle, addr, &data);
    if (status < 0) {
        return status;
    }

    data &= ~((uint8_t) bits);

    status = fps_single_write(handle, addr, data);

    return status;
}

int
fps_check_bits(fps_handle_t *handle,
               uint8_t       addr,
               int           bits)
{
    int     status = 0;
    uint8_t data;

    status = fps_single_read(handle, addr, &data);
    if (status < 0) {
        return status;
    }

    status = data & ((uint8_t) bits);

    return status;
}


////////////////////////////////////////////////////////////////////////////////
//
// Interrupt Operations
//

int
fps_enable_interrupt(fps_handle_t *handle,
                     int           event)
{
    return fps_set_bits(handle, FPS_REG_INT_CTL, event);
}

int
fps_disable_interrupt(fps_handle_t *handle,
                      int           event)
{
    return fps_clear_bits(handle, FPS_REG_INT_CTL, event);
}

int
fps_check_interrupt(fps_handle_t *handle,
                    int           event)
{
    return fps_check_bits(handle, FPS_REG_INT_EVENT, event);
}

// NOTE: Clearing interrupts is tricky. You need to switch back to Image Mode first!
int
fps_clear_interrupt(fps_handle_t *handle,
                    int           event)
{
    int     status = 0;
    uint8_t data;
    uint8_t gbl_ctl;

    // Switch to Image Mode to use SPI clock
    status = fps_single_read(handle, FPS_REG_GBL_CTL, &gbl_ctl);
    if (status < 0) {
        return status;
    }

    data = gbl_ctl & ~FPS_ENABLE_DETECT;

    status = fps_single_write(handle, FPS_REG_GBL_CTL, data);
    if (status < 0) {
        return status;
    }

    status = fps_clear_bits(handle, FPS_REG_INT_EVENT, event);
    if (status < 0) {
        return status;
    }

    // Switch back to original setting
    status = fps_single_write(handle, FPS_REG_GBL_CTL, gbl_ctl);

    return status;
}


////////////////////////////////////////////////////////////////////////////////
//
// Sensor Parameter Settings
//

static int
fps_set_cds_offset_0(fps_handle_t *handle,
                     int           mode,
                     int           offset)
{
    int     status = 0;
    uint8_t addr;
    uint8_t data;

    if ((offset < FPS_MIN_CDS_OFFSET_0) ||
        (offset > FPS_MAX_CDS_OFFSET_0)) {
        return -1;
    }

    switch (mode) {
        case FPS_IMAGE_MODE  : addr = FPS_REG_IMG_CDS_CTL_0; break;
        case FPS_DETECT_MODE : addr = FPS_REG_DET_CDS_CTL_0; break;
        default : return -1;
    }

    status = fps_single_read(handle, addr, &data);
    if (status < 0) {
        return status;
    }

    data = (data & 0x80) | (((uint8_t) offset) & 0x7F);

    status = fps_single_write(handle, addr, data);

    return status;
}

static int
fps_get_cds_offset_0(fps_handle_t *handle,
                     int           mode,
                     int          *value)
{
    int     status = 0;
    uint8_t addr;
    uint8_t data;

    switch (mode) {
        case FPS_IMAGE_MODE  : addr = FPS_REG_IMG_CDS_CTL_0; break;
        case FPS_DETECT_MODE : addr = FPS_REG_DET_CDS_CTL_0; break;
        default : return -1;
    }

    status = fps_single_read(handle, addr, &data);
    if (status < 0) {
        return status;
    }

    *value = (int) (data & 0x7F);

    return status;
}

static int
fps_set_cds_offset_1(fps_handle_t *handle,
                     int           mode,
                     int           value)
{
    int     status = 0;
    uint8_t addr[2];
    uint8_t data[2];

    if ((value < FPS_MIN_CDS_OFFSET_1) ||
        (value > FPS_MAX_CDS_OFFSET_1)) {
        return -1;
    }

    switch (mode) {
        case FPS_IMAGE_MODE :
            addr[0] = FPS_REG_IMG_CDS_CTL_0;
            addr[1] = FPS_REG_IMG_CDS_CTL_1;
            break;

        case FPS_DETECT_MODE :
            addr[0] = FPS_REG_DET_CDS_CTL_0;
            addr[1] = FPS_REG_DET_CDS_CTL_1;
            break;

        default : return -1;
    }

    status = fps_multiple_read(handle, addr, data, 2);
    if (status < 0) {
        return status;
    }

    data[0] = (uint8_t) ((value & 0x0100) >> 1);
    data[1] = (uint8_t)  (value & 0x00FF);

    status = fps_multiple_write(handle, addr, data, 2);

    return status;
}

static int
fps_get_cds_offset_1(fps_handle_t *handle,
                     int           mode,
                     int          *value)
{
    int     status = 0;
    uint8_t addr[2];
    uint8_t data[2];

    switch (mode) {
        case FPS_IMAGE_MODE :
            addr[0] = FPS_REG_IMG_CDS_CTL_0;
            addr[1] = FPS_REG_IMG_CDS_CTL_1;
            break;

        case FPS_DETECT_MODE :
            addr[0] = FPS_REG_DET_CDS_CTL_0;
            addr[1] = FPS_REG_DET_CDS_CTL_1;
            break;

        default : return -1;
    }

    status = fps_multiple_read(handle, addr, data, 2);
    if (status < 0) {
        return status;
    }

    *value = (((int) (data[0] & 0x80)) << 1) | ((int) data[1]);

    return status;
}

static int
fps_set_pga_gain_0(fps_handle_t *handle,
                   int           mode,
                   int           value)
{
    int     status = 0;
    uint8_t addr;
    uint8_t data;

    if ((value < FPS_MIN_PGA_GAIN_0) ||
        (value > FPS_MAX_PGA_GAIN_0)) {
        return -1;
    }

    switch (mode) {
        case FPS_IMAGE_MODE  : addr = FPS_REG_IMG_PGA0_CTL; break;
        case FPS_DETECT_MODE : addr = FPS_REG_DET_PGA0_CTL; break;
        default : return -1;
    }

    status = fps_single_read(handle, addr, &data);
    if (status < 0) {
        return status;
    }

    data = (data & 0xF0) | (((uint8_t) value) & 0x0F);

    status = fps_single_write(handle, addr, data);

    return status;
}

static int
fps_get_pga_gain_0(fps_handle_t *handle,
                   int           mode,
                   int          *value)
{
    int     status = 0;
    uint8_t addr;
    uint8_t data;

    switch (mode) {
        case FPS_IMAGE_MODE  : addr = FPS_REG_IMG_PGA0_CTL; break;
        case FPS_DETECT_MODE : addr = FPS_REG_DET_PGA0_CTL; break;
        default : return -1;
    }

    status = fps_single_read(handle, addr, &data);
    if (status < 0) {
        return status;
    }

    *value = (int) (data & 0x0F);

    return status;
}

static int
fps_set_pga_gain_1(fps_handle_t *handle,
                   int           mode,
                   int           value)
{
    int     status = 0;
    uint8_t addr;
    uint8_t data;

    if ((value < FPS_MIN_PGA_GAIN_1) ||
        (value > FPS_MAX_PGA_GAIN_1)) {
        return -1;
    }

    switch (mode) {
        case FPS_IMAGE_MODE  : addr = FPS_REG_IMG_PGA1_CTL; break;
        case FPS_DETECT_MODE : addr = FPS_REG_DET_PGA1_CTL; break;
        default : return -1;
    }

    status = fps_single_read(handle, addr, &data);
    if (status < 0) {
        return status;
    }

    data = (data & 0xF0) | (((uint8_t) value) & 0x0F);

    status = fps_single_write(handle, addr, data);

    return status;
}

static int
fps_get_pga_gain_1(fps_handle_t *handle,
                   int           mode,
                   int          *value)
{
    int     status = 0;
    uint8_t addr;
    uint8_t data;

    switch (mode) {
        case FPS_IMAGE_MODE  : addr = FPS_REG_IMG_PGA1_CTL; break;
        case FPS_DETECT_MODE : addr = FPS_REG_DET_PGA1_CTL; break;
        default : return -1;
    }

    status = fps_single_read(handle, addr, &data);
    if (status < 0) {
        return status;
    }

    *value = (int) (data & 0x0F);

    return status;
}

static int
fps_set_detect_threshold(fps_handle_t *handle,
                         int           value)
{
    int     status = 0;
    uint8_t data;

    if ((value < FPS_MIN_DETECT_TH) ||
        (value > FPS_MAX_DETECT_TH)) {
        return -1;
    }

    status = fps_single_read(handle, FPS_REG_V_DET_SEL, &data);
    if (status < 0) {
        return status;
    }

    data = (data & 0xC0) | (((uint8_t) value) & 0x3F);

    status = fps_single_write(handle, FPS_REG_V_DET_SEL, data);

    return status;
}

static int
fps_get_detect_threshold(fps_handle_t *handle,
                         int          *value)
{
    int     status = 0;
    uint8_t data;

    status = fps_single_read(handle, FPS_REG_V_DET_SEL, &data);
    if (status < 0) {
        return status;
    }

    *value = (int) (data & 0x3F);

    return status;
}

int
fps_set_sensor_parameter(fps_handle_t *handle,
                         int           flag,
                         int           value)
{
    int status = 0;
    int mode;
    int param;

    mode  = flag & (0xFF << 0);
    param = flag & (0xFF << 8);

    switch (param) {
        case FPS_PARAM_CDS_OFFSET_0 :
            status = fps_set_cds_offset_0(handle, mode, value);
            break;

        case FPS_PARAM_CDS_OFFSET_1 :
            status = fps_set_cds_offset_1(handle, mode, value);
            break;

        case FPS_PARAM_PGA_GAIN_0 :
            status = fps_set_pga_gain_0(handle, mode, value);
            break;

        case FPS_PARAM_PGA_GAIN_1 :
            status = fps_set_pga_gain_1(handle, mode, value);
            break;

        case FPS_PARAM_DETECT_THRESHOLD :
            status = fps_set_detect_threshold(handle, value);
            break;

        default : return -1;
    }

    return status;
}

int
fps_get_sensor_parameter(fps_handle_t *handle,
                         int           flag,
                         int          *value)
{
    int status = 0;
    int mode;
    int param;

    mode  = flag & (0xFF << 0);
    param = flag & (0xFF << 8);

    switch (param) {
        case FPS_PARAM_CDS_OFFSET_0 :
            status = fps_get_cds_offset_0(handle, mode, value);
            break;

        case FPS_PARAM_CDS_OFFSET_1 :
            status = fps_get_cds_offset_1(handle, mode, value);
            break;

        case FPS_PARAM_PGA_GAIN_0 :
            status = fps_get_pga_gain_0(handle, mode, value);
            break;

        case FPS_PARAM_PGA_GAIN_1 :
            status = fps_get_pga_gain_1(handle, mode, value);
            break;

        case FPS_PARAM_DETECT_THRESHOLD :
            status = fps_get_detect_threshold(handle, value);
            break;

        default : return -1;
    }

    return status;
}

int
fps_set_sensing_area(fps_handle_t *handle,
                     int           mode,
                     int           col_begin,
                     int           col_end,
                     int           row_begin,
                     int           row_end)
{
    int     status = 0;
    uint8_t addr[4];
    uint8_t data[4];

    switch (mode) {
        case FPS_IMAGE_MODE :
            addr[0] = FPS_REG_IMG_ROW_BEGIN;
            addr[1] = FPS_REG_IMG_ROW_END;
            addr[2] = FPS_REG_IMG_COL_BEGIN;
            addr[3] = FPS_REG_IMG_COL_END;
            break;

        case FPS_DETECT_MODE :
            addr[0] = FPS_REG_DET_ROW_BEGIN;
            addr[1] = FPS_REG_DET_ROW_END;
            addr[2] = FPS_REG_DET_COL_BEGIN;
            addr[3] = FPS_REG_DET_COL_END;
            break;

        default : return -1;
    }

    data[0] = (uint8_t) row_begin;
    data[1] = (uint8_t) row_end;
    data[2] = (uint8_t) col_begin;
    data[3] = (uint8_t) col_end;

    status = fps_multiple_write(handle, addr, data, 4);

    return status;
}

int
fps_get_sensing_area(fps_handle_t *handle,
                     int           mode,
                     int          *col_begin,
                     int          *col_end,
                     int          *row_begin,
                     int          *row_end)
{
    int     status = 0;
    uint8_t addr[4];
    uint8_t data[4];

    switch (mode) {
        case FPS_IMAGE_MODE :
            addr[0] = FPS_REG_IMG_ROW_BEGIN;
            addr[1] = FPS_REG_IMG_ROW_END;
            addr[2] = FPS_REG_IMG_COL_BEGIN;
            addr[3] = FPS_REG_IMG_COL_END;
            break;

        case FPS_DETECT_MODE :
            addr[0] = FPS_REG_DET_ROW_BEGIN;
            addr[1] = FPS_REG_DET_ROW_END;
            addr[2] = FPS_REG_DET_COL_BEGIN;
            addr[3] = FPS_REG_DET_COL_END;
            break;

        default : return -1;
    }

    status = fps_multiple_read(handle, addr, data, 4);
    if (status < 0) {
        return status;
    }

    *row_begin = (int) data[0];
    *row_end   = (int) data[1];
    *col_begin = (int) data[2];
    *col_end   = (int) data[3];

    return status;
}

int
fps_set_suspend_frames(fps_handle_t *handle,
                       int           frames)
{
    int     status = 0;
    uint8_t addr[2];
    uint8_t data[2];

    addr[0] = FPS_REG_SUSP_WAIT_F_CYC_H;
    addr[1] = FPS_REG_SUSP_WAIT_F_CYC_L;

    data[0] = (uint8_t) ((frames & 0xFF00) >> 8);
    data[1] = (uint8_t) ((frames & 0x00FF) >> 0);

    status = fps_multiple_write(handle, addr, data, 2);

    return status;
}

int
fps_get_suspend_frames(fps_handle_t *handle,
                       int          *frames)
{
    int     status = 0;
    uint8_t addr[2];
    uint8_t data[2];

    addr[0] = FPS_REG_SUSP_WAIT_F_CYC_H;
    addr[1] = FPS_REG_SUSP_WAIT_F_CYC_L;

    status = fps_multiple_read(handle, addr, data, 2);

    *frames = (((int) data[0]) << 8) | (((int) data[1]) << 0);

    return status;
}

double
fps_calculate_suspend_time(int det_size,
                           int frames)
{
    // Assume that the slowest oscillator frequency is 250KHz, i.e. 4us
    return (double) ((det_size * (1 + frames)) * 8) * 4;
}


////////////////////////////////////////////////////////////////////////////////
//
// Mode Switch
//

int
fps_switch_mode(fps_handle_t *handle,
                int           mode_new,
                int          *mode_old)
{
    int     status = 0;
    uint8_t addr[3];
    uint8_t data[3];

    addr[0] = FPS_REG_MISC_PWR_CTL_1;
    addr[1] = FPS_REG_PWR_CTL_0;
    addr[2] = FPS_REG_GBL_CTL;

    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;

    status = fps_multiple_read(handle, addr, data, 3);
    if (status < 0) {
        return status;
    }

    if (mode_old != NULL) {
        if (data[2] & FPS_ENABLE_DETECT) {
            *mode_old = FPS_DETECT_MODE;
        } else if (data[1] == FPS_PWRDWN_ALL) {
            *mode_old = FPS_POWER_DOWN_MODE;
        } else {
            *mode_old = FPS_IMAGE_MODE;
        }
    }

    // For F747B, need to enter Power Down mode first
    if (handle->chip_id == F747B_CHIP_ID) {
        data[0] |= FPS_PWRDWN_OVT;
        data[1]  = FPS_PWRDWN_ALL;
        data[2] &= ~(FPS_ENABLE_DETECT | FPS_ENABLE_PWRDWN);

        status = fps_multiple_write(handle, addr, data, 3);
        if (status < 0) {
            return status;
        }
       
        fps_sleep(5.0 * 1000);
    }

    switch (mode_new) {
        case FPS_IMAGE_MODE :
            data[0] &= ~FPS_PWRDWN_OVT;
            data[1]  = (FPS_PWRDWN_DET | FPS_PWRDWN_OSC);
            data[2] &= ~(FPS_ENABLE_DETECT | FPS_ENABLE_PWRDWN);
            break;

        case FPS_DETECT_MODE :
            data[2] |= (FPS_ENABLE_DETECT | FPS_ENABLE_PWRDWN);
            break;

        case FPS_POWER_DOWN_MODE :
            data[0] |= FPS_PWRDWN_OVT;
            data[1]  = FPS_PWRDWN_ALL;
            data[2] &= ~(FPS_ENABLE_DETECT | FPS_ENABLE_PWRDWN);
            break;

        default : return -1;
    }

    status = fps_multiple_write(handle, addr, data, 3);
    if (status < 0) {
        return status;
    }

    fps_sleep(5.0 * 1000);

    return status;
}


////////////////////////////////////////////////////////////////////////////////
//
// Image Mode Operations
//

int
fps_set_background_image(fps_handle_t *handle,
                         int           img_width,
                         int           img_height,
                         uint8_t      *img_buf)
{
    int    sensor_width;
    int    sensor_height;
    int    col_offset;
    int    row_offset;
    double pixel_sum;
    int    dst_index;
    int    src_index;
    int    c;
    int    r;

    sensor_width  = handle->sensor_width;
    sensor_height = handle->sensor_height;

    col_offset = (sensor_width  - img_width ) / 2;
    row_offset = (sensor_height - img_height) / 2;

    pixel_sum = 0.0;

    for (r = 0; r < img_height; r++) {
    for (c = 0; c < img_width;  c++) {

        src_index = r * img_width + c;
        dst_index = (row_offset + r) * sensor_width + (col_offset + c);

        handle->bkgnd_img[dst_index] = img_buf[src_index];

        pixel_sum += (double) img_buf[src_index];
    }}

    handle->bkgnd_avg = pixel_sum / (img_width * img_height);
    return 0;
}

int
fps_get_background_image(fps_handle_t *handle,
                         int           img_width,
                         int           img_height,
                         uint8_t      *img_buf)
{
    int sensor_width;
    int sensor_height;
    int col_offset;
    int row_offset;
    int dst_index;
    int src_index;
    int c;
    int r;

    sensor_width  = handle->sensor_width;
    sensor_height = handle->sensor_height;

    col_offset = (sensor_width  - img_width ) / 2;
    row_offset = (sensor_height - img_height) / 2;

    for (r = 0; r < img_height; r++) {
    for (c = 0; c < img_width;  c++) {

        src_index = (row_offset + r) * sensor_width + (col_offset + c);
        dst_index = r * img_width + c;

        img_buf[dst_index] = handle->bkgnd_img[src_index];
    }}

    return 0;
}

int
fps_get_background_average(fps_handle_t *handle,
                           double       *img_avg)
{
    *img_avg = handle->bkgnd_avg;
    return 0;
}

int
fps_get_averaged_image(fps_handle_t *handle,
                       int           img_width,
                       int           img_height,
                       int           frms_to_avg,
                       uint8_t      *img_buf,
                       double       *img_avg,
                       double       *img_var,
                       double       *img_noise)
{
    int       status = 0;
    size_t    img_size;
    uint8_t **img_array;
    uint8_t **data_array;
    double    pix_val;
    double    pix_sum;
    double    sqr_sum;
    double    var_sum;
    double   *pix_avg_buf;
    double   *pix_var_buf;
    double    pix_avg;
    double    pix_var;
    int       f;
    int       i;

    img_size = img_width * img_height;

    // Create image buffers to average
    img_array = (uint8_t **) malloc(sizeof(uint8_t *) * frms_to_avg);
    if (img_array == NULL) {
        status = -1;
        goto fps_get_averaged_image_end;
    }

    data_array = (uint8_t **) malloc(sizeof(uint8_t *) * frms_to_avg);
    if (data_array == NULL) {
        status = -1;
        goto fps_get_averaged_image_end;
    }

    for (f = 0; f < frms_to_avg; f++) {
        data_array[f] = (uint8_t *) malloc(img_size + FPS_DUMMY_PIXELS);
        if (data_array[f] == NULL) {
            status = -1;
            goto fps_get_averaged_image_end;
        }
        img_array[f] = &data_array[f][FPS_DUMMY_PIXELS];
    }

    // Create buffers to record each pixel's average and variance
    pix_avg_buf = (double *) malloc(sizeof(double) * img_size);
    if (pix_avg_buf == NULL) {
        status = -1;
        goto fps_get_averaged_image_end;
    }

    pix_var_buf = (double *) malloc(sizeof(double) * img_size);
    if (pix_var_buf == NULL) {
        status = -1;
        goto fps_get_averaged_image_end;
    }

    // Get frames to average
    for (f = 0; f < frms_to_avg; f++) {
        status = fps_get_raw_image(handle, img_width, img_height, data_array[f]);
        if (status < 0) {
            goto fps_get_averaged_image_end;
        }
    }

    // Average each frames
    for (i = 0; i < img_size; i++) {
        pix_sum = 0.0;
        sqr_sum = 0.0;
        for (f = 0; f < frms_to_avg; f++) {
            pix_val  = (double) img_array[f][i];

            pix_sum += pix_val;
            sqr_sum += SQUARE(pix_val);
        }

        pix_avg_buf[i] = pix_avg =  pix_sum / frms_to_avg;
        pix_var_buf[i] = pix_var = (sqr_sum / frms_to_avg) - SQUARE(pix_avg);
    }

    // Calculate finger image average and variance
    pix_sum = 0.0;
    sqr_sum = 0.0;
    var_sum = 0.0;
    for (i = 0; i < img_size; i++) {
        pix_val = pix_avg_buf[i];

        pix_sum +=  pix_val;
        sqr_sum += (pix_val * pix_val);
        var_sum +=  pix_var_buf[i];

        img_buf[i] = (uint8_t) pix_val;
    }

    pix_avg = pix_sum / img_size;

    if (img_avg != NULL) {
        *img_avg = pix_avg;
    }

    if (img_var != NULL) {
        *img_var = (sqr_sum / img_size) - SQUARE(pix_avg);
    }

    if (img_noise != NULL) {
        *img_noise = var_sum / img_size;
    }

fps_get_averaged_image_end :

    if (pix_avg_buf != NULL) {
        free(pix_avg_buf);
    }

    if (pix_var_buf != NULL) {
        free(pix_var_buf);
    }

    for (f = 0; f < frms_to_avg; f++) {
        if (data_array[f] != NULL) {
            free(data_array[f]);
        }
    }

    if (data_array != NULL) {
        free(data_array);
    }

    if (img_array != NULL) {
        free(img_array);
    }

    return status;
}

int
fps_get_finger_image(fps_handle_t *handle,
                     int           img_width,
                     int           img_height,
                     int           frms_to_avg,
                     uint8_t      *img_buf,
                     double       *img_dr,
                     double       *img_var,
                     double       *img_noise)
{
    int      status = 0;
    uint8_t *finger_img;
    double   finger_avg;
    double   finger_var;
    double   finger_noise;
    int      img_size;
    double   sqr_sum;
    int      i;

    finger_img = img_buf;
    status = fps_get_averaged_image(handle,
                                    img_width,
                                    img_height,
                                    frms_to_avg,
                                    finger_img,
                                    &finger_avg,
                                    &finger_var,
                                    &finger_noise);
    if (status < 0) {
        return status;
    }

    img_size = img_width * img_height;

    for (i = 0; i < img_size; i++) {
        if (finger_img[i] > handle->bkgnd_img[i]) {
            finger_img[i] = 0xFF - (finger_img[i] - handle->bkgnd_img[i]);
        } else {
            finger_img[i] = 0xFF;
        }
    }

    sqr_sum = 0.0;
    for (i = 0; i < img_size; i++) {
        sqr_sum += (double) SQUARE(finger_img[i]);
    }

    finger_var = sqr_sum / img_size - SQUARE(finger_avg);

    if (img_dr != NULL) {
        *img_dr = finger_avg - handle->bkgnd_avg;
    }

    if (img_var != NULL) {
        *img_var = finger_var;
    }

    if (img_noise != NULL) {
        *img_noise = finger_noise;
    }

    return status;
}


////////////////////////////////////////////////////////////////////////////////
//
// Detect Mode Operations
//

int
fps_set_scan_detect_method(fps_handle_t *handle,
                           int           method)
{
    handle->scan_det_method = method;
    return 0;
}

int
fps_get_scan_detect_method(fps_handle_t *handle,
                           int          *method)
{
    *method = handle->scan_det_method;
    return 0;
}

int
fps_scan_detect_event(fps_handle_t *handle,
                      double        sleep_us)
{
    int     status = 0;
    uint8_t event;

    // Disable and clear all interrupts
    FPS_DISABLE_AND_CLEAR_INTERRUPT(handle, FPS_ALL_EVENTS);

    if (handle->scan_det_method == 0) {
        // Sleep and then check detect event via reading INT_EVENT
        fps_sleep(sleep_us);

        status = event = fps_check_interrupt(handle, FPS_DETECT_EVENT);
        if (status < 0) {
            return status;
        }
    } else {
        status = fps_enable_interrupt(handle, FPS_DETECT_EVENT);
        if (status < 0) {
            return status;
        }

        status = fps_wait_event(handle, sleep_us);
        if (status < 0) {
            return status;
        }

        if (status > 0) {
            status = event = fps_check_interrupt(handle, FPS_DETECT_EVENT);
            if (status < 0) {
                return status;
            }
        }

        status = fps_disable_interrupt(handle, FPS_ALL_EVENTS);
        if (status < 0) {
            return status;
        }
    }

    return ((event & FPS_DETECT_EVENT) != 0);
}
