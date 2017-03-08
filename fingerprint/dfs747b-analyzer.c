#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <poll.h>
#include <signal.h>
#include <math.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <linux/types.h>
#include "dfs747-driver.h"

#define POWER_SOURCE_1V8_3V3  (0)
#define POWER_SOURCE_1V8_2V8  (1)
#define POWER_SOURCE_1V8_ONLY (2)
#define POWER_SOURCE_EXTERNAL (3)

#define DETECT_WINDOW_COLS    (8)
#define DETECT_WINDOW_ROWS    (8)
#define SWITCH_MODE_DELAY_US  (5000)


////////////////////////////////////////////////////////////////////////////////
//
// Global Variables
//

static int          debug_level  = 3;
static char         dev_path[]   = "/dev/dfs0";
static int          dev_fd       = 0;
static uint8_t      power_source = POWER_SOURCE_1V8_3V3;
static uint8_t      bkg_img[DFS747_SENSOR_SIZE];
static volatile int poll_stop    = 0;
static volatile int recal_done   = 0;

static uint16_t     det_cds_offset       = 0;
static uint8_t      det_detect_th        = 0;
static double       det_sleep_us         = 0.0;
static uint16_t     det_extra_cds_offset = 0;
static uint32_t     detect_cnt           = 0;
static uint32_t     recal_secs           = 0;


////////////////////////////////////////////////////////////////////////////////
//
// Utilities
//

static void
clear_console()
{
    system("clear");
}

static void
clear_line(char *line, const int size)
{
    memset(line, 0x00, size);
}

static char
get_key()
{
    char line[CHAR_CNT];

    fgets(line, sizeof(line), stdin);
    return line[0];
}

static double
get_elapsed_ms(const struct timeval *start,
                 const struct timeval *stop)
{
    double elapsed;

    if ((start == NULL) || (stop == NULL)) {
        return (double) -1;
    }

    elapsed = (double) (stop->tv_sec  * SEC + stop->tv_usec) -
              (double) (start->tv_sec * SEC + start->tv_usec);
    return elapsed / MSEC;
}

static int
save_bmp(const char     *path,
         const uint8_t  *img,
         const uint32_t size)
{
    uint8_t file_header[] = {
        0x42, 0x4D,              // 'B' 'M'
        0x36, 0x28, 0x00, 0x00,  // File size in bytes
        0x00, 0x00,              // Reserved
        0x00, 0x00,              // Reserved
        0x36, 0x04, 0x00, 0x00,  // Image offset in bytes
    };

    uint8_t info_header[] = {
        0x28, 0x00, 0x00, 0x00,  // Info header size in bytes
        0x90, 0x00, 0x00, 0x00,  // Image width in pixels
        0x40, 0x00, 0x00, 0x00,  // Image height in pixels
        0x01, 0x00,              // Number of color planes
        0x08, 0x00,              // Bits per pixel
        0x00, 0x00, 0x00, 0x00,  // Image size in bytes
        0x00, 0x00, 0x00, 0x00,  // Compression
        0xE5, 0x4C, 0x00, 0x00,  // X resolution
        0xE5, 0x4C, 0x00, 0x00,  // Y resolution
        0x00, 0x00, 0x00, 0x00,  // Number of colors
        0x00, 0x00, 0x00, 0x00   // Important colors
    };

    uint8_t color_table[256 * 4];

    FILE *fp = NULL;
    int   i;

    for (i = 0; i < 256; i++) {
        color_table[(i * 4) + 0] = i;
        color_table[(i * 4) + 1] = i;
        color_table[(i * 4) + 2] = i;
        color_table[(i * 4) + 3] = 0x00;
    }

    fp = fopen(path, "w");
    if (fp == NULL) {
        return -1;
    }

    fwrite(file_header, 1, sizeof(file_header), fp);
    fwrite(info_header, 1, sizeof(info_header), fp);
    fwrite(color_table, 1, sizeof(color_table), fp);
    fwrite(img, 1, size, fp);
    fclose(fp);

    return 0;
}

static uint8_t
find_otsu_th(const uint8_t  *img,
             const uint32_t size)
{
    uint8_t  max_th   = 0;
    double   max_otsu = 0.0;
    uint32_t nB       = 0;
    uint32_t sumB     = 0;
    uint32_t nF       = 0;
    uint32_t sumF     = 0;
    double   Wb       = 0.0;
    double   Wf       = 0.0;
    double   ub       = 0.0;
    double   uf       = 0.0;
    double   otsu     = 0.0;
    int      th;
    int      i;
    int      j;
    int      n;

    for (th = 0; th < 256; th++) {
        nB   = 0;
        sumB = 0;
        nF   = 0;
        sumF = 0;

        for (i = 2; i < (DFS747_SENSOR_ROWS - 2); i++) {
        for (j = 2; j < (DFS747_SENSOR_COLS - 2); j++) {
            n = i * DFS747_SENSOR_COLS + j;
            if (img[n] > th) {
                nF++;
                sumF += img[n];
            } else {
                nB++;
                sumB += img[n];
            }
        }}

        if ((nB == 0) || (nF == 0)) {
            continue;
        }

        Wb = (double) nB / (double) (DFS747_SENSOR_ROWS - 4) * (DFS747_SENSOR_COLS - 4);
        Wf = (double) nF / (double) (DFS747_SENSOR_ROWS - 4) * (DFS747_SENSOR_COLS - 4);

        ub = (double) sumB / (double) nB;
        uf = (double) sumF / (double) nF;

        otsu = Wb * Wf * (ub - uf) * (ub - uf);

        if (otsu > max_otsu) {
            max_otsu = otsu;
            max_th   = th;
        }
    }

    return max_th;
}

static void
find_pixel_range(const uint8_t  *img,
                 const uint32_t size,
                 uint8_t        *pix_min,
                 uint8_t        *pix_max)
{
    uint8_t min;
    uint8_t max;
    int     i;
    int     j;
    int     n;

    min = img[2 * DFS747_SENSOR_COLS + 2];
    max = img[2 * DFS747_SENSOR_COLS + 2];
    for (i = 2; i < DFS747_SENSOR_ROWS - 2; i++) {
    for (j = 2; j < DFS747_SENSOR_COLS - 2; j++) {
        n = i * DFS747_SENSOR_COLS + j;
        if (img[n] > max) max = img[n];
        if (img[n] < min) min = img[n];
    }}

    *pix_min = min;
    *pix_max = max;
}

static void ctrl_c_handler(int id)
{
    DETAIL("Hit Ctrl+C!\n");
    poll_stop = 1;
}


////////////////////////////////////////////////////////////////////////////////
//
// Sensor Access
//

static int
fps_open_sensor(const char *file,
                int        *fd)
{
    *fd = open(file, O_RDWR);
    return 0;
}

static int
fps_close_sensor(const int fd)
{
    dev_fd = 0;
    return close(fd);
}

static int
fps_reset_sensor(const int      fd,
                 const uint32_t state)
{
    int status = 0;

    struct dfs747_ioc_transfer tr = {
        .len    = state,
        .opcode = DFS747_IOC_RESET_SENSOR,
    };

    status = ioctl(fd, DFS747_IOC_MESSAGE(1), &tr);
    if (status < 0) {
        ERROR("%s(): Calling ioctl() failed! status = %0d\n", __func__, status);
    }

    return status;
}

static int
fps_set_spi_speed(const int      fd,
                  const uint32_t speed)
{
    int status = 0;

    struct dfs747_ioc_transfer tr = {
        .len    = speed,
        .opcode = DFS747_IOC_SET_CLKRATE,
    };

    status = ioctl(fd, DFS747_IOC_MESSAGE(1), &tr);
    if (status < 0) {
        ERROR("%s(): Calling ioctl() failed! status = %0d\n", __func__, status);
    }

    return status;
}

static int
fps_multiple_read(const int     fd,
                  const uint8_t *addr,
                  uint8_t       *data,
                  const uint8_t len)
{
    int     status = 0;
    int     i;
    uint8_t *tx;
    uint8_t *rx;

    if (data == NULL) {
        return -1;
    }

    tx = (uint8_t *) malloc(len);
    if (tx == NULL) {
        ERROR("%s(): malloc() failed!\n", __func__);
        goto fps_multiple_read_end;
    }

    rx = (uint8_t *) malloc(len);
    if (rx == NULL) {
        ERROR("%s(): malloc() failed!\n", __func__);
        goto fps_multiple_read_end;
    }

    for (i = 0; i < len; i++) {
        tx[i] = addr[i];
        rx[i] = 0x00;
    }

    struct dfs747_ioc_transfer tr = {
        .tx_buf = (unsigned long) tx,
        .rx_buf = (unsigned long) rx,
        .len    = len,
        .opcode = DFS747_IOC_REGISTER_MASS_READ,
    };

    status = ioctl(fd, DFS747_IOC_MESSAGE(1), &tr);
    if (status < 0) {
        ERROR("%s(): Calling ioctl() failed! status = %0d\n", __func__, status);
        goto fps_multiple_read_end;
    }

    for (i = 0; i < len; i++) {
        data[i] = rx[i];
        DETAIL("%s(): addr = 0x%02X, data = 0x%02X\n", __func__, addr[i], data[i]);
    }

fps_multiple_read_end :

    if (tx) free(tx);
    if (rx) free(rx);

    return status;
}

static int
fps_multiple_write(const int     fd,
                   const uint8_t *addr,
                   const uint8_t *data,
                   const uint8_t len)
{
    int     status = 0;
    int     i;
    uint8_t *tx;

    if ((addr == NULL) || (data == NULL)) {
        return -1;
    }

    tx = (uint8_t *) malloc(len * 2);
    if (tx == NULL) {
        ERROR("%s(): malloc() failed!\n", __func__);
        goto fps_multiple_write_end;
    }

    for (i = 0; i < len; i++) {
        tx[(i * 2) + 0] = addr[i];
        tx[(i * 2) + 1] = data[i];
    }

    struct dfs747_ioc_transfer tr = {
        .tx_buf = (unsigned long) tx,
        .len    = (len * 2),
        .opcode = DFS747_IOC_REGISTER_MASS_WRITE,
    };

    status = ioctl(fd, DFS747_IOC_MESSAGE(1), &tr);
    if (status < 0) {
        ERROR("%s(): Calling ioctl() failed! status = %0d\n", __func__, status);
        goto fps_multiple_write_end;
    }

    for (i = 0; i < len; i++) {
        DETAIL("%s(): addr = 0x%02X, data = 0x%02X\n", __func__, addr[i], data[i]);
    }

fps_multiple_write_end :

    if (tx) free(tx);

    return status;
}

static int
fps_get_one_image(const int      fd,
                  const uint32_t img_width,
                  const uint32_t img_height,
                  const uint32_t dummy_pix,
                  uint8_t        *img)
{
    int      status = 0;
    uint32_t img_size;
    uint8_t  tx[6];

    if (img == NULL) {
        return -1;
    }

    img_size = img_width * img_height + dummy_pix;

    tx[0] = img_width;
    tx[1] = img_height;
    tx[2] = dummy_pix;
    tx[3] = 0x00;
    tx[4] = 0x00;
    tx[5] = 0x00;

    // Execute get-image I/O control
    struct dfs747_ioc_transfer tr = {
        .tx_buf = (unsigned long) tx,
        .rx_buf = (unsigned long) img,
        .len    = img_size,
        .opcode = DFS747_IOC_GET_ONE_IMG
    };

    status = ioctl(fd, DFS747_IOC_MESSAGE(1), &tr);
    if (status < 0) {
        ERROR("%s(): Calling ioctl() failed! status = %0d\n", __func__, status);
        return status;
    }

    return status;
}

static int
fps_single_read(const int     fd,
                const uint8_t addr,
                uint8_t       *data)
{
    return fps_multiple_read(fd, &addr, data, 1);
}

static int
fps_single_write(const int     fd,
                 const uint8_t addr,
                 const uint8_t data)
{
    return fps_multiple_write(fd, &addr, &data, 1);
}

static int
fps_switch_mode(const int fd,
                const int mode_new,
                int       *mode_old)
{
    int     status = 0;
    uint8_t addr[3];
    uint8_t data[3];

    addr[0] = DFS747_REG_MISC_PWR_CTL_1;
    addr[1] = DFS747_REG_PWR_CTL_0;
    addr[2] = DFS747_REG_GBL_CTL;

    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;

    status = fps_multiple_read(fd, addr, data, sizeof(addr));
    if (status < 0) {
        return status;
    }

    if (mode_old != NULL) {
        if (data[2] & DFS747_ENABLE_DETECT) {
            *mode_old = DFS747_DETECT_MODE;
        } else if (data[1] == 0xFF) {
            *mode_old = DFS747_POWER_DOWN_MODE;
        } else {
            *mode_old = DFS747_IMAGE_MODE;
        }
    }

    switch (mode_new) {
        case DFS747_IMAGE_MODE : {
            data[0] &= ~DFS747_PWRDWN_OVT;
            data[1]  = (DFS747_PWRDWN_DET | DFS747_PWRDWN_OSC);
            data[2] &= ~(DFS747_ENABLE_DETECT | DFS747_ENABLE_PWRDWN);
            break;
        }

        case DFS747_DETECT_MODE : {
            data[2] |= (DFS747_ENABLE_DETECT | DFS747_ENABLE_PWRDWN);
            break;
        }

        case DFS747_POWER_DOWN_MODE : {
            data[0] |= DFS747_PWRDWN_OVT;
#if 0
            data[1]  = DFS747_PWRDWN_ALL;
#else
            data[1]  = DFS747_PWRDWN_ALL & ~DFS747_PWRDWN_BGR;
#endif
            data[2] &= ~(DFS747_ENABLE_DETECT | DFS747_ENABLE_PWRDWN);
            break;
        }

        default : return -1;
    }

    status = fps_multiple_write(fd, addr, data, sizeof(addr));
    if (status < 0) {
        return status;
    }

    usleep(SWITCH_MODE_DELAY_US);

    return status;
}

static int
fps_enable_tgen(const int fd,
                const int enable)
{
    // No need to set TGEN bit
    return 0;
}

static int
fps_scan_detect_event(const int      fd,
                      const uint8_t  detect_th,
                      const uint16_t cds_offset,
                      const double   sleep_us,
                      const int      int_enable,
                      const int      cal_detect)
{
    int            status = 0;
    uint8_t        event  = 0;
    uint8_t        addr[4];
    uint8_t        data[4];
    struct pollfd  poll_fps;

    // Switch back to Image mode
    status = fps_single_read(dev_fd, DFS747_REG_GBL_CTL, &data[0]);
    data[0] &= ~DFS747_ENABLE_DETECT;
    status = fps_single_write(dev_fd, DFS747_REG_GBL_CTL, data[0]);
    if (status < 0) {
        return status;
    }

    // Disable and clear interrupts
    addr[0] = DFS747_REG_INT_CTL;
    addr[1] = DFS747_REG_INT_EVENT;

    data[0] = 0x00;
    data[1] = 0x00;

    status = fps_multiple_write(dev_fd, addr, data, 2);
    if (status < 0) {
        return status;
    }

    // Switch back to Detect mode
    status = fps_single_read(dev_fd, DFS747_REG_GBL_CTL, &data[0]);
    data[0] |= DFS747_ENABLE_DETECT;
    status = fps_single_write(dev_fd, DFS747_REG_GBL_CTL, data[0]);
    if (status < 0) {
        return status;
    }

    if (cal_detect > 0) {
        // Set up Detect Threshold and CDS offset
        addr[0] = DFS747_REG_DET_CDS_CTL_1;
        addr[1] = DFS747_REG_DET_CDS_CTL_0;
        addr[2] = DFS747_REG_V_DET_SEL;

        status = fps_multiple_read(dev_fd, addr, data, 3);
        if (status < 0) {
            return -1;
        }

        data[0] =  (uint8_t)  (cds_offset & 0x00FF);
        data[1] = ((uint8_t) ((cds_offset & 0x0100) >> 1)) | (data[1] & 0x7F);
        data[2] = detect_th & 0x3F;

        status = fps_multiple_write(dev_fd, addr, data, 3);
        if (status < 0) {
            return -1;
        }
    }

    if (int_enable > 0) {
        // Turn on Detect interrupt
        status = fps_single_write(dev_fd, DFS747_REG_INT_CTL, DFS747_DETECT_EVENT);
        if (status < 0) {
            return -1;
        }

        poll_fps.fd      = dev_fd;
        poll_fps.events  = POLLIN;
        poll_fps.revents = 0;
        status = poll(&poll_fps, 1, (int) (sleep_us / 1000));
        if (status < 0) {
            if ((poll_stop == 0) && (recal_done == 0)) {
                ERROR("%s(): Calling poll() failed! status = %0d\n", __func__, status);
                return -1;
            }
        }
        DETAIL("%s(): poll_fps.revents = %0d\n", __func__, poll_fps.revents);

        if (poll_fps.revents != 0) {
            status = fps_single_read(dev_fd, DFS747_REG_INT_EVENT, &event);
            if (status < 0) {
                return -1;
            }
            DETAIL("%s(): event = %02X\n", __func__, event);
        }

        // Turn off Detect interrupt
        status = fps_single_write(dev_fd, DFS747_REG_INT_CTL, 0x00);
        if (status < 0) {
            return -1;
        }
    } else {
        usleep(sleep_us);

        // Read if Detect interrupt happens
        status = fps_single_read(dev_fd, DFS747_REG_INT_EVENT, &event);
        if (status < 0) {
            return -1;
        }
    }

    return ((event & DFS747_DETECT_EVENT) != 0);
}

static int
fps_search_cds_offset(const int      fd,
                      const uint8_t  detect_th,
                      const uint16_t upper,
                      const uint16_t lower,
                      const double   sleep_us,
                      const uint32_t scan_limit,
                      const int      int_enable,
                      uint16_t       *cds_offset)
{
    int      status = 0;
    uint16_t cds_upper;
    uint16_t cds_middle;
    uint16_t cds_lower;
    uint32_t scan_cnt;
    uint8_t  addr[2];
    uint8_t  data[2];

    cds_upper  = upper;
    cds_middle = (upper + lower) / 2;
    cds_lower  = lower;

    while ((cds_upper - cds_lower) > 1) {
        DEBUG("%s(): CDS Offset range = 0x%03X : 0x%03X : 0x%03X\n", __func__,
               cds_upper, cds_middle, cds_lower);

        status = fps_scan_detect_event(fd, detect_th, cds_middle, sleep_us, int_enable, 1);
        for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
            status = fps_scan_detect_event(fd, detect_th, cds_middle, sleep_us, int_enable, 0);

            // Finger-on detected
            if (status > 0) {
                break;
            }

            // Error
            if (status < 0) {
                return status;
            }
        }

        if (scan_cnt == scan_limit) {
            cds_upper = cds_middle;
        } else {
            cds_lower = cds_middle;
        }

        cds_middle = (cds_upper + cds_lower) / 2;
    }

    *cds_offset = cds_upper;

    addr[0] = DFS747_REG_DET_CDS_CTL_0;
    addr[1] = DFS747_REG_DET_CDS_CTL_1;

    status = fps_multiple_read(fd, addr, data, 2);
    if (status < 0) {
        return status;
    }

    data[0] = (data[0] & 0x7F) | ((uint8_t) ((*cds_offset & 0x0100) >> 1));
    data[1] = (uint8_t) (*cds_offset & 0x00FF);

    status = fps_multiple_write(fd, addr, data, 2);
    if (status < 0) {
        return status;
    }

    if (*cds_offset == upper) {
        WARN("%s(): Maximum CDS Offset reached!\n", __func__);
    }

    if (*cds_offset == lower) {
        WARN("%s(): Minimum CDS Offset reached!\n", __func__);
    }

    return 0;
}

static int
fps_search_detect_threshold(const int      fd,
                            const uint16_t cds_offset,
                            const uint8_t  upper,
                            const uint8_t  lower,
                            const double   sleep_us,
                            const uint32_t scan_limit,
                            const int      int_enable,
                            uint8_t        *detect_th)
{
    int      status = 0;
    uint8_t  det_upper;
    uint8_t  det_middle;
    uint8_t  det_lower;
    uint32_t scan_cnt;

    det_upper  = upper;
    det_middle = (upper + lower) / 2;
    det_lower  = lower;

    while ((det_upper - det_lower) > 1) {
        DEBUG("%s(): Detect Threshold range = 0x%02X : 0x%02X : 0x%02X\n", __func__,
               det_upper, det_middle, det_lower);

        status = fps_scan_detect_event(fd, det_middle, cds_offset, sleep_us, int_enable, 1);
        for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
            status = fps_scan_detect_event(fd, det_middle, cds_offset, sleep_us, int_enable, 0);

            // Finger-on detected
            if (status > 0) {
                break;
            }

            // Error
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

    *detect_th = det_upper;

    status = fps_single_write(fd, DFS747_REG_V_DET_SEL, *detect_th);
    if (status < 0) {
        return status;
    }

    if (*detect_th == upper) {
        WARN("%s(): Maximum Detect Threshold reached!\n", __func__);
    }

    if (*detect_th == lower) {
        WARN("%s(): Minimum Detect Threshold reached!\n", __func__);
    }

    return 0;
}

static int
fps_search_detect_window_from_bkgnd_image(const uint8_t col_scan_begin,
                                          const uint8_t col_scan_end,
                                          uint8_t       *row_scan_begin,
                                          uint8_t       *row_scan_end,
                                          uint16_t      *extra_cds_offset,
                                          double        *win_dev)
{
    int      status        = 0;
    uint32_t img_height    = DFS747_SENSOR_ROWS;
    double   pix           = 0.0;
    double   pix_sum       = 0.0;
    double   sqr_sum       = 0.0;
    double   avg           = 0.0;
    double   dev           = 0.0;
    uint32_t col_scan_cnt  = 0;
    uint32_t row_scan_cnt  = 0;
    uint32_t win_scan_cnt  = 0;
    double   *all_wins_dev = NULL;
    double   *all_wins_avg = NULL;
    int      w;
    int      rb;
    int      r;
    int      c;
    uint32_t skip_rows      = 16;
    uint32_t skip_rows_half = 0;

    col_scan_cnt = col_scan_end - col_scan_begin + 1;
    row_scan_cnt = DETECT_WINDOW_ROWS;
    win_scan_cnt = img_height - skip_rows - row_scan_cnt + 1;

    skip_rows_half = skip_rows / 2;

    all_wins_avg = (double *) malloc(sizeof(double) * win_scan_cnt);
    if (all_wins_avg == NULL) {
        status = -1;
        goto fps_search_detect_window_from_bkgnd_image_error;
    }

    all_wins_dev = (double *) malloc(sizeof(double) * win_scan_cnt);
    if (all_wins_dev == NULL) {
        status = -1;
        goto fps_search_detect_window_from_bkgnd_image_error;
    }

    for (w = 0; w < win_scan_cnt; w++) {
        all_wins_avg[w] = 0.0;
        all_wins_dev[w] = 0.0;
    }

    for (rb = skip_rows_half; rb < (skip_rows_half + win_scan_cnt); rb++) {
        pix_sum = 0.0;
        sqr_sum = 0.0;
        for (r = rb; r < (rb + row_scan_cnt); r++) {
            for (c = col_scan_begin; c <= col_scan_end; c++) {
                pix = (double) bkg_img[r * DFS747_SENSOR_COLS + c];
                pix_sum +=  pix;
                sqr_sum += (pix * pix);
            }
        }

        avg =  pix_sum / (row_scan_cnt * col_scan_cnt);
        dev = (sqr_sum / (row_scan_cnt * col_scan_cnt)) - (avg * avg);

        DEBUG("%s(): Row (Begin/End) = %0d/%0d, Deviation = %0.3f\n",
              __func__, rb, (rb + row_scan_cnt - 1), dev);

        if (dev > all_wins_dev[rb - skip_rows_half]) {
            all_wins_dev[rb - skip_rows_half] = dev;
            all_wins_avg[rb - skip_rows_half] = avg;
        }
    }

    for (rb = skip_rows_half; rb < (skip_rows_half + win_scan_cnt); rb++) {
        DEBUG("%s(): Row (Begin/End) %0d/%0d, Avg. = %0.3f, Dev. = %0.3f\n",
              __func__, rb, (rb + row_scan_cnt - 1),
              all_wins_avg[rb - skip_rows_half],
              all_wins_dev[rb - skip_rows_half]);
    }

    *row_scan_begin = skip_rows_half;
    *row_scan_end   = skip_rows_half + row_scan_cnt - 1;
    *win_dev        = all_wins_dev[0];

    for (w = 1; w < win_scan_cnt; w++) {
        if ((all_wins_dev[w] < *win_dev) && (*win_dev != 0)) {
            *row_scan_begin = w + skip_rows_half;
            *row_scan_end   = w + skip_rows_half + row_scan_cnt - 1;
            *win_dev        = all_wins_dev[w];
        }
    }

    // TODO: May use image characteristic to find the extra_cds_offset
    if (extra_cds_offset != NULL) {
        *extra_cds_offset = 0x05;
    }

    if (*win_dev >= 50.0) {
        DEBUG("%s(): Too bad to search a good detect window!\n", __func__);
        status = -1;
        goto fps_search_detect_window_from_bkgnd_image_error;
    }

    DEBUG("%s(): Sel. Row (Begin/End) = %0d/%0d, Dev. = %0.3f, Extra CDS Offset = 0x%03X\n",
          __func__, *row_scan_begin, *row_scan_end, *win_dev,
          ((extra_cds_offset == NULL) ? 0 : *extra_cds_offset));

fps_search_detect_window_from_bkgnd_image_error:

    if (all_wins_dev) free(all_wins_dev);
    if (all_wins_avg) free(all_wins_avg);

    return status;
}

static int
fps_search_detect_window(const int      fd,
                         const uint8_t  col_scan_begin,
                         const uint8_t  col_scan_end,
                         const uint32_t repeat_times,
                         uint16_t       *extra_cds_offset,
                         uint8_t        *row_scan_begin,
                         uint8_t        *row_scan_end,
                         double         *win_dev)
{
    int      status        = 0;
    uint8_t  **raw_buf     = NULL;
    uint8_t  **img_buf     = NULL;
    uint8_t  *avg_img      = NULL;
    uint8_t  avg_frame     = 1;
    uint32_t img_width     = DFS747_SENSOR_COLS;
    uint32_t img_height    = DFS747_SENSOR_ROWS;
    uint32_t img_size      = DFS747_SENSOR_SIZE;
    double   pix           = 0.0;
    double   pix_sum       = 0.0;
    double   sqr_sum       = 0.0;
    double   avg           = 0.0;
    double   dev           = 0.0;
    uint32_t col_scan_cnt  = 0;
    uint32_t row_scan_cnt  = 0;
    uint32_t win_scan_cnt  = 0;
    double   *all_wins_dev = NULL;
    double   *all_wins_avg = NULL;
    int      t;
    int      f;
    int      w;
    int      rb;
    int      r;
    int      c;
    int      p;
    char     file[CHAR_CNT];

    col_scan_cnt = col_scan_end - col_scan_begin + 1;
    row_scan_cnt = col_scan_cnt;
    win_scan_cnt = img_height - 2 - col_scan_cnt + 1;

    // NOTE: Assume the sensor is in Image Mode before this call...

    // Switch CDS out as ADC singal source
    status = fps_single_write(fd, DFS747_REG_TEST_ANA, 0x03);
    if (status < 0) {
        return -1;
    }

    // Create image buffers to average later
    img_buf = (uint8_t **) malloc(sizeof(uint8_t *) * avg_frame);
    if (img_buf == NULL) {
        status = -1;
        goto fps_search_detect_window_error;
    }

    raw_buf = (uint8_t **) malloc(sizeof(uint8_t *) * avg_frame);
    if (raw_buf == NULL) {
        status = -1;
        goto fps_search_detect_window_error;
    }

    for (f = 0; f < avg_frame; f++) {
        raw_buf[f] = (uint8_t *) malloc(img_size + DFS747_DUMMY_PIXELS);
        if (raw_buf[f] == NULL) {
            status = -1;
            goto fps_search_detect_window_error;
        }
        img_buf[f] = &raw_buf[f][DFS747_DUMMY_PIXELS];
    }

    avg_img = (uint8_t *) malloc(img_size);
    if (avg_img == NULL) {
        status = -1;
        goto fps_search_detect_window_error;
    }

    all_wins_avg = (double *) malloc(sizeof(double) * win_scan_cnt);
    if (all_wins_avg == NULL) {
        status = -1;
        goto fps_search_detect_window_error;
    }

    all_wins_dev = (double *) malloc(sizeof(double) * win_scan_cnt);
    if (all_wins_dev == NULL) {
        status = -1;
        goto fps_search_detect_window_error;
    }

    for (w = 0; w < win_scan_cnt; w++) {
        all_wins_avg[w] = 0.0;
        all_wins_dev[w] = 0.0;
    }

    for (t = 0; t < repeat_times; t++) {
        for (f = 0; f < avg_frame; f++) {
            // Clear all pending events
            status = fps_single_write(fd, DFS747_REG_INT_EVENT, 0x00);
            if (status < 0) {
                goto fps_search_detect_window_error;
            }

            // Turn on TGEN
            status = fps_enable_tgen(dev_fd, 1);
            if (status < 0) {
                goto fps_search_detect_window_error;
            }

            // Get a frame
            status = fps_get_one_image(dev_fd,
                                       img_width,
                                       img_height,
                                       DFS747_DUMMY_PIXELS,
                                       raw_buf[f]);
            if (status < 0) {
                goto fps_search_detect_window_error;
            }

            // Turn off TGEN
            status = fps_enable_tgen(dev_fd, 0);
            if (status < 0) {
                goto fps_search_detect_window_error;
            }
        }

        // Average each frame
        for (p = 0; p < img_size; p++) {
            pix_sum = 0.0;
            for (f = 0; f < avg_frame; f++) {
                pix_sum += (double) img_buf[f][p];
            }

            avg_img[p] = (uint8_t) (pix_sum / avg_frame);
        }

        sprintf(file, "detect_search.bmp");
        status = save_bmp(file, avg_img, img_size);
        if (status < 0) {
            goto fps_search_detect_window_error;
        }

        for (rb = 1; rb < (1 + win_scan_cnt); rb++) {
            pix_sum = 0.0;
            sqr_sum = 0.0;
            for (r = rb; r < (rb + row_scan_cnt); r++) {
                for (c = col_scan_begin; c <= col_scan_end; c++) {
                    pix = (double) avg_img[r * DFS747_SENSOR_COLS + c];
                    pix_sum +=  pix;
                    sqr_sum += (pix * pix);
                }
            }

            avg =  pix_sum / (row_scan_cnt * col_scan_cnt);
            dev = (sqr_sum / (row_scan_cnt * col_scan_cnt)) - (avg * avg);

            DEBUG("%s(): Row (Begin/End) = %0d/%0d, Deviation = %0.3f\n",
                  __func__, rb, (rb + row_scan_cnt - 1), dev);

            if (dev > all_wins_dev[rb - 1]) {
                all_wins_dev[rb - 1] = dev;
                all_wins_avg[rb - 1] = avg;
            }
        }
    }

    for (rb = 1; rb < (1 + win_scan_cnt); rb++) {
        DEBUG("%s(): Row (Begin/End) %0d/%0d, Avg. = %0.3f, Dev. = %0.3f\n",
              __func__, rb, (rb + row_scan_cnt - 1),
              all_wins_avg[rb - 1], all_wins_dev[rb - 1]);
    }

    *row_scan_begin = 1;
    *row_scan_end   = 1 + row_scan_cnt - 1;
    *win_dev        = all_wins_dev[0];

    for (w = 1; w < win_scan_cnt; w++) {
        if ((all_wins_dev[w] < *win_dev) && (*win_dev != 0)) {
            *row_scan_begin = w + 1;
            *row_scan_end   = w + 1 + row_scan_cnt - 1;
            *win_dev        = all_wins_dev[w];
        }
    }

#if 0
    *extra_cds_offset = (uint16_t) (((((*win_dev) * 1.8 * 1000) / 256) * 1.2) / 2);
#else
    *extra_cds_offset = (uint16_t) (((((*win_dev) * 22.2 * 1000) / 256) * 1.2) / 2);
#endif
    if (*extra_cds_offset == 0) {
        *extra_cds_offset = 1;
    }

    if (*win_dev >= 10.0) {
        DEBUG("%s(): Too bad to search a good detect window!\n", __func__);
        status = -1;
        goto fps_search_detect_window_error;
    }

    DEBUG("%s(): Sel. Row (Begin/End) = %0d/%0d, Dev. = %0.3f, Extra CDS Offset = 0x%03X\n",
          __func__, *row_scan_begin, *row_scan_end, *win_dev, *extra_cds_offset);

fps_search_detect_window_error:

    // Switch PGA out as ADC signal source
    (void) fps_single_write(fd, DFS747_REG_TEST_ANA, 0x00);

    // Free allocated buffers
    if (avg_img) free(avg_img);
    for (f = 0; f < avg_frame; f++) {
        if (raw_buf[f]) free(raw_buf[f]);
    }
    if (raw_buf)      free(raw_buf);
    if (img_buf)      free(img_buf);
    if (all_wins_dev) free(all_wins_dev);
    if (all_wins_avg) free(all_wins_avg);

    return status;
}

static int
fps_search_detect_line(const int      fd,
                       const uint8_t  col_scan_begin,
                       const uint8_t  col_scan_end,
                       const uint8_t  row_scan_begin,
                       const uint8_t  row_scan_end,
                       const uint32_t repeat_times,
                       uint16_t       *extra_cds_offset,
                       uint8_t        *row_found,
                       double         *row_dev)
{
    int      status        = 0;
    uint8_t  **raw_buf     = NULL;
    uint8_t  **img_buf     = NULL;
    uint8_t  *avg_img      = NULL;
    uint8_t  avg_frame     = 1;
    uint32_t img_width     = DFS747_SENSOR_COLS;
    uint32_t img_height    = DFS747_SENSOR_ROWS;
    uint32_t img_size      = DFS747_SENSOR_SIZE;
    double   pix           = 0.0;
    double   pix_sum       = 0.0;
    double   sqr_sum       = 0.0;
    double   avg           = 0.0;
    double   dev           = 0.0;
    uint32_t row_scan_cnt  = 0;
    uint32_t col_scan_cnt  = 0;
    double   *all_rows_dev = NULL;
    double   *all_rows_avg = NULL;
    int      t;
    int      f;
    int      r;
    int      c;
    int      p;
    char     file[CHAR_CNT];

    row_scan_cnt = row_scan_end - row_scan_begin + 1;
    col_scan_cnt = col_scan_end - col_scan_begin + 1;

    // NOTE: Assume the sensor is in Image Mode before this call...

    // Switch CDS out as ADC singal source
    status = fps_single_write(fd, DFS747_REG_TEST_ANA, 0x03);
    if (status < 0) {
        return -1;
    }

    // Create image buffers to average later
    img_buf = (uint8_t **) malloc(sizeof(uint8_t *) * avg_frame);
    if (img_buf == NULL) {
        status = -1;
        goto fps_search_detect_line_error;
    }

    raw_buf = (uint8_t **) malloc(sizeof(uint8_t *) * avg_frame);
    if (raw_buf == NULL) {
        status = -1;
        goto fps_search_detect_line_error;
    }

    for (f = 0; f < avg_frame; f++) {
        raw_buf[f] = (uint8_t *) malloc(img_size + DFS747_DUMMY_PIXELS);
        if (raw_buf[f] == NULL) {
            status = -1;
            goto fps_search_detect_line_error;
        }
        img_buf[f] = &raw_buf[f][DFS747_DUMMY_PIXELS];
    }

    avg_img = (uint8_t *) malloc(img_size);
    if (avg_img == NULL) {
        status = -1;
        goto fps_search_detect_line_error;
    }

    all_rows_avg = (double *) malloc(sizeof(double) * row_scan_cnt);
    if (all_rows_avg == NULL) {
        status = -1;
        goto fps_search_detect_line_error;
    }

    all_rows_dev = (double *) malloc(sizeof(double) * row_scan_cnt);
    if (all_rows_dev == NULL) {
        status = -1;
        goto fps_search_detect_line_error;
    }

    for (r = 0; r < row_scan_cnt; r++) {
        all_rows_avg[r] = 0.0;
        all_rows_dev[r] = 0.0;
    }

    for (t = 0; t < repeat_times; t++) {
        for (f = 0; f < avg_frame; f++) {
            // Clear all pending events
            status = fps_single_write(fd, DFS747_REG_INT_EVENT, 0x00);
            if (status < 0) {
                goto fps_search_detect_line_error;
            }

            // Turn on TGEN
            status = fps_enable_tgen(dev_fd, 1);
            if (status < 0) {
                goto fps_search_detect_line_error;
            }

            // Get a frame
            status = fps_get_one_image(dev_fd,
                                       img_width,
                                       img_height,
                                       DFS747_DUMMY_PIXELS,
                                       raw_buf[f]);
            if (status < 0) {
                goto fps_search_detect_line_error;
            }

            // Turn off TGEN
            status = fps_enable_tgen(dev_fd, 0);
            if (status < 0) {
                goto fps_search_detect_line_error;
            }
        }

        // Average each frame
        for (p = 0; p < img_size; p++) {
            pix_sum = 0.0;
            for (f = 0; f < avg_frame; f++) {
                pix_sum += (double) img_buf[f][p];
            }

            avg_img[p] = (uint8_t) (pix_sum / avg_frame);
        }

        sprintf(file, "detect_search.bmp");
        status = save_bmp(file, avg_img, img_size);
        if (status < 0) {
            goto fps_search_detect_line_error;
        }

        for (r = row_scan_begin; r <= row_scan_end; r++) {
            pix_sum = 0.0;
            sqr_sum = 0.0;
            for (c = col_scan_begin; c <= col_scan_end; c++) {
                pix = (double) avg_img[r * DFS747_SENSOR_COLS + c];
                pix_sum +=  pix;
                sqr_sum += (pix * pix);
            }

            avg =  pix_sum / col_scan_cnt;
            dev = (sqr_sum / col_scan_cnt) - (avg * avg);

            DEBUG("%s(): Row = %0d, Deviation = %0.3f\n", __func__, r, dev);

            if (dev > all_rows_dev[r - row_scan_begin]) {
                all_rows_dev[r - row_scan_begin] = dev;
                all_rows_avg[r - row_scan_begin] = avg;
            }
        }
    }

    for (r = 0; r < row_scan_cnt; r++) {
        DEBUG("%s(): Row %0d, Avg. = %0.3f, Dev. = %0.3f\n",
              __func__, (r + row_scan_begin), all_rows_avg[r], all_rows_dev[r]);
    }

    *row_found = row_scan_begin;
    *row_dev   = all_rows_dev[0];

    for (r = 1; r <= (row_scan_end - row_scan_begin); r++) {
        if ((all_rows_dev[r] < *row_dev) && (*row_dev != 0)) {
            *row_found = r + row_scan_begin;
            *row_dev   = all_rows_dev[r];
        }
    }

    *extra_cds_offset = (uint16_t) (((((*row_dev) * 1.8 * 1000) / 256) * 1.2) / 2);
    if (*extra_cds_offset == 0) {
        *extra_cds_offset = 1;
    }

    if (*row_dev >= 10.0) {
        DEBUG("%s(): Too bad to search a good detect window!\n", __func__);
        status = -1;
        goto fps_search_detect_line_error;
    }

    DEBUG("%s(): Sel. Row = %0d, Dev. = %0.3f, Extra CDS Offset = 0x%03X\n",
          __func__, *row_found, *row_dev, *extra_cds_offset);

fps_search_detect_line_error:

    // Switch PGA out as ADC signal source
    (void) fps_single_write(fd, DFS747_REG_TEST_ANA, 0x00);

    // Free allocated buffers
    if (avg_img) free(avg_img);
    for (f = 0; f < avg_frame; f++) {
        if (raw_buf[f]) free(raw_buf[f]);
    }
    if (raw_buf)      free(raw_buf);
    if (img_buf)      free(img_buf);
    if (all_rows_dev) free(all_rows_dev);
    if (all_rows_avg) free(all_rows_avg);

    return status;
}


////////////////////////////////////////////////////////////////////////////////
//
// Menu Functions
//

int
open_sensor()
{
    int status = 0;

    while (1) {
        clear_console();

        printf("\n");
        printf("=============\n");
        printf(" Open Sensor \n");
        printf("=============\n");
        printf("\n");
        printf("Result:\n");
        printf("\n");

        printf("    Opening sensor...\n");

        status = fps_open_sensor(dev_path, &dev_fd);
        if (status < 0) {
            printf("    Failed!\n");
        } else {
            printf("    Done!\n");
        }

        printf("\n");
        printf("Press ENTER key to continue... ");
        (void) getchar();

        break;
    }

    return status;
}

int
close_sensor()
{
    int status = 0;

    while (1) {
        clear_console();

        printf("\n");
        printf("==============\n");
        printf(" Close Sensor \n");
        printf("==============\n");
        printf("\n");
        printf("Result:\n");
        printf("\n");

        printf("    Closing sensor...\n");

        status = fps_close_sensor(dev_fd);
        if (status < 0) {
            printf("    Failed!\n");
        } else {
            printf("    Done!\n");
        }

        printf("\n");
        printf("Press ENTER key to continue... ");
        (void) getchar();

        break;
    }

    return status;
}

int
init_sensor()
{
    int     status     = 0;
    char    key;
    char    line[CHAR_CNT];
    char    *arg;
    int     user_input = 0;
    uint8_t addr[4];
    uint8_t data[4];

    memset(bkg_img, 0x00, sizeof(bkg_img));

    while (1) {
        clear_console();

        printf("\n");
        printf("===================\n");
        printf(" Initialize Sensor \n");
        printf("===================\n");
        printf("\n");
        printf("    'p' src - Specify where the power source comes from.\n");
        printf("                src = Power source identifiers.         \n");
        printf("                      0: 1.8V + 3.3V                    \n");
        printf("                      1: 1.8V + 2.8V                    \n");
        printf("                      2: 1.8V only                      \n");
        printf("                      3: External                       \n");
        printf("                This number must be decimal.            \n");
        printf("    'g'     - Get the power source setting.             \n");
        printf("    ENTER   - Do the initialization.                    \n");
        printf("    'q'     - Back to main menu.                        \n");
        printf("\n");
        printf("Pleae enter: ");

        clear_line(line, sizeof(line));
        fgets(line, sizeof(line), stdin);
        key = line[0];

        if (key == 'q') {
            break;
        }

        printf("\n");
        printf("Result:\n");
        printf("\n");

        if (key == 'p') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if ((user_input >= POWER_SOURCE_1V8_3V3 ) &&
                (user_input <= POWER_SOURCE_EXTERNAL)) {
                power_source = (uint8_t) user_input;
                goto init_sensor_show_options;
            } else {
                printf("    ERROR: Power source must be >= 0 and <= 2!\n");
            }
        }

        if (key == 'g') {

init_sensor_show_options :

            printf("    Power Source = %0d (%s)\n", power_source,
                   (power_source == POWER_SOURCE_1V8_3V3)  ? "1.8V + 3.3V" :
                   (power_source == POWER_SOURCE_1V8_2V8)  ? "1.8V + 2.8V" :
                   (power_source == POWER_SOURCE_1V8_ONLY) ? "1.8V only"   :
                   (power_source == POWER_SOURCE_EXTERNAL) ? "External"    : "Unknown");
        }

        if (key == '\n') {
            printf("    Initializing...\n");

            // Set SPI clock speed
            status = fps_set_spi_speed(dev_fd, 8 * 1000 * 1000);
            if (status < 0) {
                return status;
            }

            // Disable and clear interrupts
            addr[0] = DFS747_REG_INT_CTL;
            addr[1] = DFS747_REG_INT_EVENT;

            data[0] = 0x00;
            data[1] = 0x00;

            status = fps_multiple_write(dev_fd, addr, data, 2);
            if (status < 0) {
                return status;
            }

            // Enter power down mode
            status = fps_switch_mode(dev_fd, DFS747_POWER_DOWN_MODE, NULL);
            if (status < 0) {
                return status;
            }

            // Specify the power source configuration
            addr[0] = 0x05;
            addr[1] = 0x06;

            switch (power_source) {
                case POWER_SOURCE_1V8_3V3 :
                    data[0] = 0x10;
                    data[1] = 0x3C;
                    break;

                case POWER_SOURCE_1V8_2V8 :
                    data[0] = 0xFC;
                    data[1] = 0x3C;
                    break;

                case POWER_SOURCE_1V8_ONLY :
                    data[0] = 0xFC;
                    data[1] = 0x3C;
                    break;

                case POWER_SOURCE_EXTERNAL :
                    data[0] = 0x00;
                    data[1] = 0x00;
                    break;
            }

            status = fps_multiple_write(dev_fd, addr, data, 2);
            if (status < 0) {
                return status;
            }
            usleep(50 * MSEC);

            // Settings for sensing enhancement
            addr[0] = 0x0E;
            addr[1] = 0x09;
            addr[2] = 0x1E;
            addr[3] = 0x04;

            data[0] = 0x20;
            data[1] = 0x03;
            data[2] = 0x1F;
            data[3] = 0x20;

            status = fps_multiple_write(dev_fd, addr, data, 4);
            if (status < 0) {
                return status;
            }

            // Enter image mode
            status = fps_switch_mode(dev_fd, DFS747_IMAGE_MODE, NULL);
            if (status < 0) {
                return status;
            }

            printf("    Done!\n");
        }

        printf("\n");
        printf("Press ENTER key to continue... ");
        (void) getchar();
    }

    return status;
}

int
read_register()
{
    int     status = 0;
    char    key;
    char    line[CHAR_CNT];
    char    *arg;
    uint8_t addr[DFS747_REG_COUNT];
    uint8_t data[DFS747_REG_COUNT];
    uint8_t len = 0;
    int     i;

    for (i = 0; i < DFS747_REG_COUNT; i++) {
        addr[i] = 0xFF;
        data[i] = 0x00;
    }

    while (1) {
        clear_console();

        printf("\n");
        printf("===============\n");
        printf(" Read Register \n");
        printf("===============\n");
        printf("\n");
        printf("    'r' a1 a2 ... aN - Read register addr1 to addrN one by one.\n");
        printf("                       These address must be seperated by spa- \n");
        printf("                       ces and must be hex. number.            \n");
        printf("    ENTER            - Repeat this read again.                 \n");
        printf("    'q'              - Back to main menu.                      \n");
        printf("\n");
        printf("Pleae enter: ");

        clear_line(line, sizeof(line));
        fgets(line, sizeof(line), stdin);
        key = line[0];

        if (key == 'q') {
            break;
        }

        if (key == 'r') {
            for (i = 0; i < DFS747_REG_COUNT; i++) {
                addr[i] = 0xFF;
            }

            arg = strtok(line, " ");
            for (i = 0; i < DFS747_REG_COUNT; i++) {
                arg = strtok(NULL, " ");
                if (arg == NULL) {
                    break;
                }
                addr[i] = strtol(arg, NULL, 16);
            }

            len = i;
        }

        // Before reading registers, the addresses must be specified.
        if (addr[0] != 0xFF) {

            status = fps_multiple_read(dev_fd, addr, data, len);
            if (status < 0) {
                return status;
            }

            printf("\n");
            printf("Result:\n");
            printf("\n");
            for (i = 0; addr[i] != 0xFF; i++) {
                printf("    Addr = 0x%02X, Data = 0x%02X\n", addr[i], data[i]);
            }
            printf("\n");

            printf("Press ENTER key to continue... ");
            (void) get_key();
        }
    }

    return status;
}

int
write_register()
{
    int     status = 0;
    char    key    = 0;
    char    line[CHAR_CNT];
    char    *arg;
    uint8_t len = 0;
    uint8_t addr[DFS747_REG_COUNT];
    uint8_t data[DFS747_REG_COUNT];
    int     i;

    for (i = 0; i < DFS747_REG_COUNT; i++) {
        addr[i] = 0xFF;
        data[i] = 0x00;
    }

    while (1) {
        clear_console();

        printf("\n");
        printf("================\n");
        printf(" Write Register \n");
        printf("================\n");
        printf("\n");
        printf("    'w' a1 d1 ... aN dN - Write register a1 to aN one by one.\n");
        printf("                          These address must be seperated by \n");
        printf("                          spaces, and must be hex. number.   \n");
        printf("    ENTER               - Repeat this write again.           \n");
        printf("    'q'                 - Back to main menu.                 \n");
        printf("\n");
        printf("Pleae enter: ");

        clear_line(line, sizeof(line));
        fgets(line, sizeof(line), stdin);
        key = line[0];

        if (key == 'q') {
            break;
        }

        if (key == 'w') {
            for (i = 0; i < sizeof(addr); i++) {
                addr[i] = 0xFF;
                data[i] = 0x00;
            }

            len = i;

            arg = strtok(line, " ");
            for (i = 0; i < sizeof(addr); i++) {
                arg = strtok(NULL, " ");
                if (arg == NULL) {
                    break;
                }
                addr[i] = strtol(arg, NULL, 16);

                arg = strtok(NULL, " ");
                if (arg == NULL) {
                    break;
                }
                data[i] = strtol(arg, NULL, 16);
            }
        }

        // Before writing registers, the addresses must be specified.
        if (addr[0] != 0xFF) {

            status = fps_multiple_write(dev_fd, addr, data, len);
            if (status < 0) {
                return status;
            }

            printf("\n");
            printf("Result:\n");
            printf("\n");
            for (i = 0; addr[i] != 0xFF; i++) {
                printf("    Addr = 0x%02X, Data = 0x%02X\n", addr[i], data[i]);
            }
            printf("\n");

            printf("Press ENTER key to continue... ");
            (void) getchar();
        }
    }

    return status;
}

int
dump_register()
{
    int     status = 0;
    char    key    = 0;
    uint8_t addr[DFS747_REG_COUNT];
    uint8_t data[DFS747_REG_COUNT];
    int     i;

    while (1) {
        clear_console();

        printf("\n");
        printf("====================\n");
        printf(" Dump All Registers \n");
        printf("====================\n");
        printf("\n");
        printf("Result:\n");
        printf("\n");

        for (i = 0; i < DFS747_REG_COUNT; i++) {
            addr[i] = i;
            data[i] = 0;
        }

        status = fps_multiple_read(dev_fd, addr, data, DFS747_REG_COUNT);
        if (status < 0) {
            return status;
        }

        for (i = 0; i < DFS747_REG_COUNT; i++) {
            printf("    Addr = 0x%02X, Data = 0x%02X\n", addr[i], data[i]);
        }

        printf("\n");
        printf("Press 'q' to back to main menu, or ENTER to dump again: ");

        key = get_key();

        if (key == '\n') {
            continue;
        }

        if (key == 'q') {
            break;
        }
    }

    return status;
}

int
calibrate_image2()
{
    int            status              = 0;
    char           key                 = 0;
    char           line[CHAR_CNT];
    char           *arg                = NULL;
    int            user_input_i[2]     = {0, 0};
    double         user_input_f        = 0.0;
    uint8_t        upper_bond          = 240;
    uint8_t        lower_bond          = 10;
    uint8_t        row_scan_begin      = DFS747_SENSOR_ROWS * 1/4;
    uint8_t        row_scan_end        = DFS747_SENSOR_ROWS * 3/4 - 1;
    uint8_t        col_scan_begin      = DFS747_SENSOR_COLS * 1/4;
    uint8_t        col_scan_end        = DFS747_SENSOR_COLS * 3/4 - 1;
    uint32_t       scan_size           = DFS747_SENSOR_SIZE * 1/4;
    double         ratio               = 0.05;
    uint8_t        addr[4];
    uint8_t        data[4];
    int            mode_old            = 0;
    uint8_t        row_begin           = 0;
    uint8_t        row_end             = DFS747_SENSOR_ROWS - 1;
    uint8_t        col_begin           = 0;
    uint8_t        col_end             = DFS747_SENSOR_COLS - 1;
    uint32_t       img_width           = DFS747_SENSOR_COLS;
    uint32_t       img_height          = DFS747_SENSOR_ROWS;
    uint32_t       img_size            = DFS747_SENSOR_SIZE;
    uint8_t        avg_frame           = 1;
    uint8_t        **raw_buf           = NULL;
    uint8_t        **img_buf           = NULL;
    uint8_t        *avg_img            = NULL;
    double         sum                 = 0.0;
    uint16_t       cds_offset          = DFS747_MAX_CDS_OFFSET;
    uint8_t        pga_gain            = DFS747_MAX_PGA_GAIN;
    uint32_t       pix_cnt_over_upper  = 0;
    uint32_t       pix_cnt_under_lower = 0;
    int            img_too_dark        = 1;
    int            img_too_bright      = 1;
    struct timeval start_time;
    struct timeval stop_time;
    double         elapsed             = 0.0;
    int            cal_success         = 0;
    int            i;
    int            f;
    int            r;
    int            c;

    while (1) {
        clear_console();

        printf("\n");
        printf("============================\n");
        printf(" Calibrate Image (Method 2) \n");
        printf("============================\n");
        printf("\n");
        printf("    'b' ub lb - Set target upper and lower bond of calibrated pixels.\n");
        printf("                  ub = upper bond.                                   \n");
        printf("                  lb = lower bond.                                   \n");
        printf("                These numbers must be decimal.                       \n");
        printf("    't' ratio - Specify a ratio of over-boundary pixels.             \n");
        printf("                This number must be a <=1.0 floating point number.   \n");
        printf("    'r' rb re - Specify begin and end row index to be scanned.       \n");
        printf("                  rb = row index to begin scanning                   \n");
        printf("                  re = row index to end scanning                     \n");
        printf("                These numbers must be decimal.                       \n");
        printf("    'c' cb ce - Specify begin and end column index to be scanned.    \n");
        printf("                  cb = column index to begin scanning                \n");
        printf("                  ce = column index to end scanning                  \n");
        printf("                These numbers must be decimal.                       \n");
        printf("    'a' frm   - Specify how many frames to average.                  \n");
        printf("                  frm = number of frames to average an image.        \n");
        printf("                This number must be decimal.                         \n");
        printf("    'g'       - Get current image calibration settings.              \n");
        printf("    ENTER     - Do image calibration based on current settings.      \n");
        printf("    'q'       - Back to main menu.                                   \n");
        printf("\n");
        printf("Pleae enter: ");

        clear_line(line, sizeof(line));
        fgets(line, sizeof(line), stdin);
        key = line[0];

        if (key == 'q') {
            break;
        }

        printf("\n");
        printf("Result:\n");
        printf("\n");

        if (key == 'b') {
            arg             = strtok(line, " ");
            arg             = strtok(NULL, " ");
            user_input_i[0] = strtol(arg, NULL, 10);
            arg             = strtok(NULL, " ");
            user_input_i[1] = strtol(arg, NULL, 10);

            if (user_input_i[0] >= user_input_i[1]) {
                upper_bond = (uint8_t) user_input_i[0];
                lower_bond = (uint8_t) user_input_i[1];
                goto calibrate_image2_show_options;
            } else {
                printf("    ERROR: Upper bond must be >= lower bond!\n");
            }
        }

        if (key == 't') {
            arg          = strtok(line, " ");
            arg          = strtok(NULL, " ");
            user_input_f = strtod(arg, NULL);

            if ((user_input_f > 0.0) && (user_input_f <= 1.0)) {
                ratio = user_input_f;
                goto calibrate_image2_show_options;
            } else {
                printf("    ERROR: Ratio must be in range 0.0 < ratio <= 1.0!\n");
            }
        }

        if (key == 'r') {
            arg             = strtok(line, " ");
            arg             = strtok(NULL, " ");
            user_input_i[0] = strtol(arg, NULL, 10);
            arg             = strtok(NULL, " ");
            user_input_i[1] = strtol(arg, NULL, 10);

            if (user_input_i[1] >= user_input_i[0]) {
                row_scan_begin = (uint8_t) user_input_i[0];
                row_scan_end   = (uint8_t) user_input_i[1];
                goto calibrate_image2_show_options;
            } else {
                printf("    ERROR: Row scan end point must be >= begin point!\n");
            }
        }

        if (key == 'c') {
            arg             = strtok(line, " ");
            arg             = strtok(NULL, " ");
            user_input_i[0] = strtol(arg, NULL, 10);
            arg             = strtok(NULL, " ");
            user_input_i[1] = strtol(arg, NULL, 10);

            if (user_input_i[1] >= user_input_i[0]) {
                col_scan_begin = (uint8_t) user_input_i[0];
                col_scan_end   = (uint8_t) user_input_i[1];
                goto calibrate_image2_show_options;
            } else {
                printf("    ERROR: Column scan end point must be >= begin point!\n");
            }
        }

        if (key == 'a') {
            arg             = strtok(line, " ");
            arg             = strtok(NULL, " ");
            user_input_i[0] = strtol(arg, NULL, 10);

            if (user_input_i[0] >= 1) {
                avg_frame = (uint32_t) user_input_i[0];
                goto calibrate_image2_show_options;
            } else {
                printf("    ERROR: Number of frame to average must be >= 1!\n");
            }
        }

        if (key == 'g') {

calibrate_image2_show_options :

            printf("    Calibration Bond (Upper/Lower) = %0d/%0d\n", upper_bond, lower_bond);
            printf("    Row Scan (Begin/End)           = %0d/%0d\n", row_scan_begin, row_scan_end);
            printf("    Column Scan (Begin/End)        = %0d/%0d\n", col_scan_begin, col_scan_end);
            printf("    Number of Frames to Average    = %0d\n",     avg_frame);
            printf("    Ratio                          = %0.3f\n",   ratio);
        }

        if (key == '\n') {
            printf("    Calibrating...\n");

            // Set image window
            addr[0] = DFS747_REG_IMG_ROW_BEGIN;
            addr[1] = DFS747_REG_IMG_ROW_END;
            addr[2] = DFS747_REG_IMG_COL_BEGIN;
            addr[3] = DFS747_REG_IMG_COL_END;

            data[0] = row_begin;
            data[1] = row_end;
            data[2] = col_begin;
            data[3] = col_end;

            status = fps_multiple_write(dev_fd, addr, data, 4);
            if (status < 0) {
                goto calibrate_image2_error;
            }

            img_width  = col_end - col_begin + 1;
            img_height = row_end - row_begin + 1;
            img_size   = img_width * img_height;

            raw_buf = (uint8_t **) malloc(sizeof(uint8_t *) * avg_frame);
            if (raw_buf == NULL) {
                status = -1;
                goto calibrate_image2_error;
            }

            img_buf = (uint8_t **) malloc(sizeof(uint8_t *) * avg_frame);
            if (img_buf == NULL) {
                status = -1;
                goto calibrate_image2_error;
            }

            // Create image buffers to average
            for (f = 0; f < avg_frame; f++) {
                raw_buf[f] = (uint8_t *) malloc(img_size + DFS747_DUMMY_PIXELS);
                if (raw_buf[f] == NULL) {
                    status = -1;
                    goto calibrate_image2_error;
                }
                img_buf[f] = &raw_buf[f][DFS747_DUMMY_PIXELS];
            }

            // Create an image buffer to store averaged result
            avg_img = (uint8_t *) malloc(img_size);
            if (avg_img == NULL) {
                status = -1;
                goto calibrate_image2_error;
            }

            addr[0] = DFS747_REG_IMG_CDS_CTL_0;
            addr[1] = DFS747_REG_IMG_CDS_CTL_1;
            addr[2] = DFS747_REG_IMG_PGA1_CTL;

#if 0
            cds_offset = DFS747_MAX_CDS_OFFSET;
            pga_gain   = DFS747_MAX_PGA_GAIN;
#else
            cds_offset = DFS747_MAX_CDS_OFFSET - 0x80;
            pga_gain   = DFS747_MAX_PGA_GAIN   - 0x02;
#endif

            // Switch to image mode
#if 0
            status = fps_switch_mode(dev_fd, DFS747_IMAGE_MODE, &mode_old);
            if (status < 0) {
                goto calibrate_image2_error;
            }
#else
            status = fps_switch_mode(dev_fd, DFS747_POWER_DOWN_MODE, &mode_old);
            status = fps_switch_mode(dev_fd, DFS747_IMAGE_MODE, NULL);
            if (status < 0) {
                goto calibrate_image2_error;
            }
#endif

            status = gettimeofday(&start_time, NULL);
            if (status < 0) {
                goto calibrate_image2_error;
            }

            while (1) {
                DEBUG("CDS Offset = 0x%03X, PGA Gain = 0x%02X\n", cds_offset, pga_gain);

                // Set CDS Offset and PGA Gain
                status = fps_multiple_read(dev_fd, addr, data, 3);
                if (status < 0) {
                    goto calibrate_image2_error;
                }

                data[0] = ((uint8_t) ((cds_offset & 0x0100) >> 1)) | (data[0] & 0x7F);
                data[1] =  (uint8_t)  (cds_offset & 0x00FF);
                data[2] = (pga_gain & 0x0F) | (data[2] & 0xF0);

                status = fps_multiple_write(dev_fd, addr, data, 3);
                if (status < 0) {
                    goto calibrate_image2_error;
                }

                for (f = 0; f < avg_frame; f++) {
                    // Clear all pending events
                    status = fps_single_write(dev_fd, DFS747_REG_INT_EVENT, 0x00);
                    if (status < 0) {
                        goto calibrate_image2_error;
                    }

                    // Turn on TGEN
                    status = fps_enable_tgen(dev_fd, 1);
                    if (status < 0) {
                        goto calibrate_image2_error;
                    }

                    // Get a frame
                    status = fps_get_one_image(dev_fd,
                                               img_width,
                                               img_height,
                                               DFS747_DUMMY_PIXELS,
                                               raw_buf[f]);
                    if (status < 0) {
                        goto calibrate_image2_error;
                    }

                    // Turn off TGEN
                    status = fps_enable_tgen(dev_fd, 0);
                    if (status < 0) {
                        goto calibrate_image2_error;
                    }
                }

                // Average each frame
                for (i = 0; i < img_size; i++) {
                    sum = 0.0;
                    for (f = 0; f < avg_frame; f++) {
                        sum += (double) img_buf[f][i];
                    }

                    avg_img[i] = (uint8_t) (sum / avg_frame);
                }

                // Do calibration
                pix_cnt_over_upper  = 0;
                pix_cnt_under_lower = 0;

                for (r = row_scan_begin; r <= row_scan_end; r++) {
                for (c = col_scan_begin; c <= col_scan_end; c++) {
                    if (avg_img[r * DFS747_SENSOR_COLS + c] > upper_bond) {
                        pix_cnt_over_upper++;
                    }
                    if (avg_img[r * DFS747_SENSOR_COLS + c] < lower_bond) {
                        pix_cnt_under_lower++;
                    }
                }}

                scan_size = (row_scan_end - row_scan_begin + 1) *
                            (col_scan_end - col_scan_begin + 1);

                img_too_bright = (pix_cnt_over_upper  >= (uint32_t) (((double) scan_size) * ratio));
                img_too_dark   = (pix_cnt_under_lower >= (uint32_t) (((double) scan_size) * ratio));

                DEBUG("pix_cnt_over_upper = %0d, pix_cnt_under_lower = %0d\n", pix_cnt_over_upper, pix_cnt_under_lower);
                DEBUG("Scan Size = %0d, Too Bright = %0d, Too Dark = %0d\n", scan_size, img_too_bright, img_too_dark);

                if ((img_too_bright == 0) && (img_too_dark == 0)) {
                    cal_success = 1;
                    break;
                }

                if (img_too_bright > 0) {
                    if (cds_offset != DFS747_MAX_CDS_OFFSET) {
                        cds_offset++;
                        continue;
                    }

                    if (pga_gain != 5) {
                        pga_gain--;
                        cds_offset = DFS747_MAX_CDS_OFFSET;
                        continue;
                    }
                }

                if (img_too_dark > 0) {
                    if (cds_offset != DFS747_MIN_CDS_OFFSET) {
                        cds_offset--;
                        continue;
                    }

                    if (pga_gain != DFS747_MAX_PGA_GAIN) {
                        pga_gain++;
                        cds_offset = DFS747_MAX_CDS_OFFSET;
                        continue;
                    }
                }

                cal_success = 0;
                break;
            }

            status = gettimeofday(&stop_time, NULL);
            if (status < 0) {
                goto calibrate_image2_error;
            }

            elapsed = get_elapsed_ms(&start_time, &stop_time);
            if (elapsed < 0) {
                goto calibrate_image2_error;
            }

            if (cal_success > 0) {
                memcpy(bkg_img, avg_img, img_size);

                printf("    Done!\n");
                printf("\n");
                printf("    CDS Offset = 0x%03X\n", cds_offset);
                printf("    PGA Gain   = 0x%02X\n", pga_gain);
                printf("\n");
                printf("    Time elapsed = %0.3f ms\n", elapsed);
            } else {
                printf("    Failed!\n");
            }

            // Free allocated buffers
            if (avg_img) free(avg_img);
            for (f = 0; f < avg_frame; f++) {
                if (raw_buf[f]) free(raw_buf[f]);
            }
            if (raw_buf) free(raw_buf);
            if (img_buf) free(img_buf);
        }

        printf("\n");
        printf("Press ENTER key to continue... ");
        (void) getchar();
    }

    return status;

calibrate_image2_error :

    // Free allocated buffers
    if (avg_img) free(avg_img);
    for (f = 0; f < avg_frame; f++) {
        if (raw_buf[f]) free(raw_buf[f]);
    }
    if (raw_buf) free(raw_buf);
    if (img_buf) free(img_buf);

    printf("\n");
    printf("    Failed!\n");
    printf("\n");

    return status;
}

int
calibrate_image1()
{
    int            status          = 0;
    char           key             = 0;
    char           line[CHAR_CNT];
    char           *arg            = NULL;
    int            user_input_i[2] = {0, 0};
    uint8_t        upper_bond      = 240;
    uint8_t        lower_bond      = 10;
    uint8_t        first_row       = 2;
    uint8_t        middle_row      = DFS747_SENSOR_ROWS / 2;
    uint8_t        last_row        = DFS747_SENSOR_ROWS - 2;
    uint8_t        col_scan_begin  = 4;
    uint8_t        col_scan_end    = DFS747_SENSOR_COLS - 4;
    uint8_t        addr[4];
    uint8_t        data[4];
    int            mode_old        = 0;
    uint8_t        row_begin       = 0;
    uint8_t        row_end         = DFS747_SENSOR_ROWS - 1;
    uint8_t        col_begin       = 0;
    uint8_t        col_end         = DFS747_SENSOR_COLS - 1;
    uint32_t       img_width       = DFS747_SENSOR_COLS;
    uint32_t       img_height      = DFS747_SENSOR_ROWS;
    uint32_t       img_size        = (DFS747_SENSOR_COLS * DFS747_SENSOR_ROWS);
    uint8_t        avg_frame       = 4;
    uint8_t        **raw_buf       = NULL;
    uint8_t        **img_buf       = NULL;
    uint8_t        *avg_img        = NULL;
    double         sum             = 0.0;
    uint16_t       cds_offset      = DFS747_MAX_CDS_OFFSET;
    uint8_t        pga_gain        = DFS747_MAX_PGA_GAIN;
    int            pix_gt_lower    = 1;
    int            pix_lt_upper    = 1;
    struct timeval start_time;
    struct timeval stop_time;
    double         elapsed         = 0.0;
    int            cal_success     = 0;
    int            i;
    int            f;

    while (1) {
        clear_console();

        printf("\n");
        printf("============================\n");
        printf(" Calibrate Image (Method 1) \n");
        printf("============================\n");
        printf("\n");
        printf("    'b' ub lb    - Set target upper and lower bond of calibrated pixels.\n");
        printf("                     ub = upper bond.                                   \n");
        printf("                     lb = lower bond.                                   \n");
        printf("                   These numbers must be decimal.                       \n");
        printf("    'r' rf rm rl - Specify rows to be scanned.                          \n");
        printf("                     rf = first row                                     \n");
        printf("                     rm = middle row                                    \n");
        printf("                     rl = last row                                      \n");
        printf("                   These numbers must be decimal.                       \n");
        printf("    'c' cb ce    - Specify begin and end column index to be scanned.    \n");
        printf("                     cb = column index to begin scanning                \n");
        printf("                     ce = column index to end scanning                  \n");
        printf("                   These numbers must be decimal.                       \n");
        printf("    'a' frm      - Specify how many frames to average.                  \n");
        printf("                     frm = number of frames to average an image.        \n");
        printf("                   This number must be decimal.                         \n");
        printf("    'g'          - Get current image calibration settings.              \n");
        printf("    ENTER        - Do image calibration based on current settings.      \n");
        printf("    'q'          - Back to main menu.                                   \n");
        printf("\n");
        printf("Pleae enter: ");

        clear_line(line, sizeof(line));
        fgets(line, sizeof(line), stdin);
        key = line[0];

        if (key == 'q') {
            break;
        }

        printf("\n");
        printf("Result:\n");
        printf("\n");

        if (key == 'b') {
            arg             = strtok(line, " ");
            arg             = strtok(NULL, " ");
            user_input_i[0] = strtol(arg, NULL, 10);
            arg             = strtok(NULL, " ");
            user_input_i[1] = strtol(arg, NULL, 10);

            if (user_input_i[0] >= user_input_i[1]) {
                upper_bond = (uint8_t) user_input_i[0];
                lower_bond = (uint8_t) user_input_i[1];
                goto calibrate_image1_show_options;
            } else {
                printf("    ERROR: Upper bond must be >= lower bond!\n");
            }
        }

        if (key == 'r') {
            arg             = strtok(line, " ");
            arg             = strtok(NULL, " ");
            user_input_i[0] = strtol(arg, NULL, 10);
            arg             = strtok(NULL, " ");
            user_input_i[1] = strtol(arg, NULL, 10);
            arg             = strtok(NULL, " ");
            user_input_i[2] = strtol(arg, NULL, 10);

            if ((user_input_i[0] <= user_input_i[1]) &&
                (user_input_i[1] <= user_input_i[2])) {
                first_row  = (uint8_t) user_input_i[0];
                middle_row = (uint8_t) user_input_i[1];
                last_row   = (uint8_t) user_input_i[2];
                goto calibrate_image1_show_options;
            } else {
                printf("    ERROR: Must satisfy First < Middle < Last!\n");
            }
        }

        if (key == 'c') {
            arg             = strtok(line, " ");
            arg             = strtok(NULL, " ");
            user_input_i[0] = strtol(arg, NULL, 10);
            arg             = strtok(NULL, " ");
            user_input_i[1] = strtol(arg, NULL, 10);

            if (user_input_i[1] >= user_input_i[0]) {
                col_scan_begin = (uint8_t) user_input_i[0];
                col_scan_end   = (uint8_t) user_input_i[1];
                goto calibrate_image1_show_options;
            } else {
                printf("    ERROR: Column scan end point must be >= begin point!\n");
            }
        }

        if (key == 'a') {
            arg             = strtok(line, " ");
            arg             = strtok(NULL, " ");
            user_input_i[0] = strtol(arg, NULL, 10);

            if (user_input_i[0] >= 1) {
                avg_frame = (uint32_t) user_input_i[0];
                goto calibrate_image1_show_options;
            } else {
                printf("    ERROR: Number of frame to average must be >= 1!\n");
            }
        }

        if (key == 'g') {

calibrate_image1_show_options :

            printf("    Calibration Bond (Upper/Lower) = %0d/%0d\n",     upper_bond, lower_bond);
            printf("    Rows (First/Middle/Last)       = %0d/%0d/%0d\n", first_row, middle_row, last_row);
            printf("    Column Scan (Begin/End)        = %0d/%0d\n",     col_scan_begin, col_scan_end);
            printf("    Number of Frames to Average    = %0d\n",         avg_frame);
        }

        if (key == '\n') {
            printf("    Calibrating...\n");

            // Set image window
            addr[0] = DFS747_REG_IMG_ROW_BEGIN;
            addr[1] = DFS747_REG_IMG_ROW_END;
            addr[2] = DFS747_REG_IMG_COL_BEGIN;
            addr[3] = DFS747_REG_IMG_COL_END;

            data[0] = row_begin;
            data[1] = row_end;
            data[2] = col_begin;
            data[3] = col_end;

            status = fps_multiple_write(dev_fd, addr, data, 4);
            if (status < 0) {
                goto calibrate_image1_error;
            }

            img_width  = col_end - col_begin + 1;
            img_height = row_end - row_begin + 1;
            img_size   = img_width * img_height;

            raw_buf = (uint8_t **) malloc(sizeof(uint8_t *) * avg_frame);
            if (raw_buf == NULL) {
                status = -1;
                goto calibrate_image1_error;
            }

            img_buf = (uint8_t **) malloc(sizeof(uint8_t *) * avg_frame);
            if (img_buf == NULL) {
                status = -1;
                goto calibrate_image1_error;
            }

            // Create image buffers to average
            for (f = 0; f < avg_frame; f++) {
                raw_buf[f] = (uint8_t *) malloc(img_size + DFS747_DUMMY_PIXELS);
                if (raw_buf[f] == NULL) {
                    status = -1;
                    goto calibrate_image1_error;
                }
                img_buf[f] = &raw_buf[f][DFS747_DUMMY_PIXELS];
            }

            // Create an image buffer to store averaged result
            avg_img = (uint8_t *) malloc(img_size);
            if (avg_img == NULL) {
                status = -1;
                goto calibrate_image1_error;
            }

            addr[0] = DFS747_REG_IMG_CDS_CTL_0;
            addr[1] = DFS747_REG_IMG_CDS_CTL_1;
            addr[2] = DFS747_REG_IMG_PGA1_CTL;

            cds_offset = DFS747_MAX_CDS_OFFSET;
            pga_gain   = DFS747_MAX_PGA_GAIN;

            // Switch to image mode
#if 0
            status = fps_switch_mode(dev_fd, DFS747_IMAGE_MODE, &mode_old);
            if (status < 0) {
                goto calibrate_image1_error;
            }
#else
            status = fps_switch_mode(dev_fd, DFS747_POWER_DOWN_MODE, &mode_old);
            status = fps_switch_mode(dev_fd, DFS747_IMAGE_MODE, NULL);
            if (status < 0) {
                goto calibrate_image1_error;
            }
#endif

            status = gettimeofday(&start_time, NULL);
            if (status < 0) {
                goto calibrate_image1_error;
            }

            while (1) {
                DEBUG("CDS Offset = 0x%03X, PGA Gain = 0x%02X\n", cds_offset, pga_gain);

                // Set CDS Offset and PGA Gain
                status = fps_multiple_read(dev_fd, addr, data, 3);
                if (status < 0) {
                    goto calibrate_image1_error;
                }

                data[0] = ((uint8_t) ((cds_offset & 0x0100) >> 1)) | (data[0] & 0x7F);
                data[1] =  (uint8_t)  (cds_offset & 0x00FF);
                data[2] = (pga_gain & 0x0F) | (data[2] & 0xF0);

                status = fps_multiple_write(dev_fd, addr, data, 3);
                if (status < 0) {
                    goto calibrate_image1_error;
                }

                for (f = 0; f < avg_frame; f++) {
                    // Clear all pending events
                    status = fps_single_write(dev_fd, DFS747_REG_INT_EVENT, 0x00);
                    if (status < 0) {
                        goto calibrate_image1_error;
                    }

                    // Turn on TGEN
                    status = fps_enable_tgen(dev_fd, 1);
                    if (status < 0) {
                        goto calibrate_image1_error;
                    }

                    // Get a frame
                    status = fps_get_one_image(dev_fd,
                                               img_width,
                                               img_height,
                                               DFS747_DUMMY_PIXELS,
                                               raw_buf[f]);
                    if (status < 0) {
                        goto calibrate_image1_error;
                    }

                    // Turn off TGEN
                    status = fps_enable_tgen(dev_fd, 0);
                    if (status < 0) {
                        goto calibrate_image1_error;
                    }
                }

                // Average each frame
                for (i = 0; i < img_size; i++) {
                    sum = 0.0;
                    for (f = 0; f < avg_frame; f++) {
                        sum += (double) img_buf[f][i];
                    }

                    avg_img[i] = (uint8_t) (sum / avg_frame);
                }

                pix_gt_lower = 1;
                pix_lt_upper = 1;

                for (i = col_scan_begin; i <= col_scan_end; i++) {
                    if (avg_img[first_row  * DFS747_SENSOR_ROWS + i] > upper_bond) {
                        pix_lt_upper = 0;
                    }

                    if (avg_img[middle_row * DFS747_SENSOR_ROWS + i] < lower_bond) {
                        pix_gt_lower = 0;
                    }

                    if (avg_img[last_row   * DFS747_SENSOR_ROWS + i] > upper_bond) {
                        pix_lt_upper = 0;
                    }
                }

                if ((pix_gt_lower  == 1) && (pix_lt_upper == 1)) {
                    cal_success = 1;
                    break;
                }

                if ((pga_gain == 0) || (cds_offset == 0)) {
                    cal_success = 0;
                    break;
                }

                if (pix_gt_lower == 0) {
                    if (cds_offset == DFS747_MIN_CDS_OFFSET) {
                        cal_success = 0;
                        break;
                    }

                    cds_offset--;
                    continue;
                }

                if (pix_lt_upper == 0) {
                    if (pga_gain == DFS747_MIN_PGA_GAIN) {
                        cal_success = 0;
                        break;
                    }

                    pga_gain--;
                    continue;
                }
            }

            status = gettimeofday(&stop_time, NULL);
            if (status < 0) {
                goto calibrate_image1_error;
            }

            elapsed = get_elapsed_ms(&start_time, &stop_time);
            if (elapsed < 0) {
                goto calibrate_image1_error;
            }

            if (cal_success > 0) {
                memcpy(bkg_img, avg_img, img_size);

                printf("    Done!\n");
                printf("\n");
                printf("    CDS Offset = 0x%03X\n", cds_offset);
                printf("    PGA Gain   = 0x%02X\n", pga_gain);
                printf("\n");
                printf("    Time elapsed = %0.3f ms\n", elapsed);
            } else {
                printf("    Failed!\n");
            }

            // Free allocated buffers
            if (avg_img) free(avg_img);
            for (f = 0; f < avg_frame; f++) {
                if (raw_buf[f]) free(raw_buf[f]);
            }
            if (raw_buf) free(raw_buf);
            if (img_buf) free(img_buf);
        }

        printf("Press ENTER key to continue... ");
        (void) getchar();
    }

    return status;

calibrate_image1_error :

    // Free allocated buffers
    if (avg_img) free(avg_img);
    for (f = 0; f < avg_frame; f++) {
        if (raw_buf[f]) free(raw_buf[f]);
    }
    if (raw_buf) free(raw_buf);
    if (img_buf) free(img_buf);

    printf("\n");
    printf("    Failed!\n");
    printf("\n");

    return status;
}

int
calibrate_detect4()
{
    int            status           = 0;
    char           key              = 0;
    char           line[CHAR_CNT];
    char           *arg             = NULL;
    int            user_input       = 0;
    uint16_t       det_frame        = 4;
    int            scan_limit       = 8;
    int            scan_cnt         = 0;
    uint8_t        addr[4];
    uint8_t        data[4];
    int            mode_old         = 0;
    double         win_dev          = 0.0;
    uint8_t        row_begin        = (DFS747_SENSOR_ROWS / 2) - (DETECT_WINDOW_ROWS / 2);
    uint8_t        row_end          = (DFS747_SENSOR_ROWS / 2) + (DETECT_WINDOW_ROWS / 2) - 1;
    uint8_t        col_begin        = (DFS747_SENSOR_COLS / 2) - (DETECT_WINDOW_COLS / 2);
    uint8_t        col_end          = (DFS747_SENSOR_COLS / 2) + (DETECT_WINDOW_COLS / 2) - 1;
    size_t         det_size         = 16;
    uint16_t       cds_offset       = DFS747_MAX_CDS_OFFSET;
    uint8_t        detect_th        = DFS747_MAX_DETECT_TH;
    double         sleep_us         = 0.0;
    struct timeval start_time;
    struct timeval stop_time;
    double         elapsed          = 0.0;
    uint16_t       extra_cds_offset = 6;
    uint8_t        extra_detect_th  = 1;
    int            int_enable       = 1;
    int            fine_tune_enable = 1;
    int            cal_success      = 1;
    int            double_sense     = 0;
    int            finger_on        = 0;
    int            finger_off       = 0;
    uint8_t        det_max;
    uint8_t        det_min;
    uint8_t        det_upper;
    uint8_t        det_middle;
    uint8_t        det_lower;
    uint16_t       cds_max;
    uint16_t       cds_min;
    uint16_t       cds_upper;
    uint16_t       cds_middle;
    uint16_t       cds_lower;
    int            retry_limit      = 10;
    int            retry_cnt        = 0;
    int            sleep_mul        = 2;

    while (1) {
        clear_console();

        printf("\n");
        printf("=============================\n");
        printf(" Calibrate Detect (Method 4) \n");
        printf("=============================\n");
        printf("\n");
        printf("NOTE: This require Image Calibration first!                   \n");
        printf("                                                              \n");
        printf("    's' frm - Specify suspend interval.                       \n");
        printf("                frm = Suspend interval, and uint is frames.   \n");
        printf("              This number must be decimal.                    \n");
        printf("    't' scn - Specify detect scan times in a iteration.       \n");
        printf("                scn = Detect scan times.                      \n");
        printf("              This number must be decimal.                    \n");
        printf("    'm' mul - Specify the multiplier for the sleep time.      \n");
        printf("                mul = Multiplier.                             \n");
        printf("              This number must be decimal.                    \n");
        printf("    'X' det - Specify extra Detect Threshold.                 \n");
        printf("                det = Detect Threshold alue.                  \n");
        printf("              This number must be decimal.                    \n");
        printf("    'x' cds - Specify extra CDS offset.                       \n");
        printf("                cds = CDS offset value.                       \n");
        printf("              This number must be decimal.                    \n");
        printf("    'g'     - Get current detect calibration settings.        \n");
        printf("    ENTER   - Do detect calibration based on current settings.\n");
        printf("    'q'     - Back to main menu.                              \n");
        printf("\n");
        printf("Pleae enter: ");

        clear_line(line, sizeof(line));
        fgets(line, sizeof(line), stdin);
        key = line[0];

        if (key == 'q') {
            break;
        }

        printf("\n");
        printf("Result:\n");
        printf("\n");

        if (key == 's') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 1) {
                det_frame = (uint16_t) user_input;
                goto calibrate_detect4_show_options;
            } else {
                printf("    ERROR: Suspend interval must be >= 1!\n");
            }
        }

        if (key == 't') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 1) {
                scan_limit = (uint32_t) user_input;
                goto calibrate_detect4_show_options;
            } else {
                printf("    ERROR: Scan limit must be >= 1!\n");
            }
        }

        if (key == 'f') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 0) {
                fine_tune_enable = user_input;
                goto calibrate_detect4_show_options;
            } else {
                printf("    ERROR: This option must be either 0 or 1!\n");
            }
        }

        if (key == 'm') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 1) {
                sleep_mul = (int) user_input;
                goto calibrate_detect4_show_options;
            } else {
                printf("    ERROR: Sleep time multiplier must be >= 1!\n");
            }
        }

        if (key == 'X') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 0) {
                extra_detect_th = (uint8_t) user_input;
                goto calibrate_detect4_show_options;
            } else {
                printf("    ERROR: Extra Detect Threshold must be >= 0!\n");
            }
        }

        if (key == 'x') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 0) {
                extra_cds_offset = (uint16_t) user_input;
                goto calibrate_detect4_show_options;
            } else {
                printf("    ERROR: Extra CDS offset must be >= 0!\n");
            }
        }

        if (key == 'd') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 0) {
                double_sense = user_input;
                goto calibrate_detect4_show_options;
            } else {
                printf("    ERROR: This option must be either 0 or 1!\n");
            }
        }

        if (key == 'g') {

calibrate_detect4_show_options :

            printf("    Suspend Interval          = %0d frames\n", det_frame);
            printf("    Scan Limit                = %0d\n",        scan_limit);
            printf("    Sleep Time Multiplier     = %0d\n",        sleep_mul);
            printf("    Extra Detect Threshold    = %0d\n",        extra_detect_th);
            printf("    Extra CDS Offset          = %0d\n",        extra_cds_offset);
        }

        if (key == '\n') {
            printf("    Calibrating...\n");

            // Disable all interrupts
            status = fps_single_write(dev_fd, DFS747_REG_INT_CTL, 0x00);
            if (status < 0) {
                goto calibrate_detect4_error;
            }

            // Copy the settings from the Image Calibration
            addr[0] = DFS747_REG_IMG_CDS_CTL_0;
            addr[1] = DFS747_REG_IMG_CDS_CTL_1;
            addr[2] = DFS747_REG_IMG_PGA0_CTL;
            addr[3] = DFS747_REG_IMG_PGA1_CTL;

            status = fps_multiple_read(dev_fd, addr, data, 4);
            if (status < 0) {
                goto calibrate_detect4_error;
            }

            addr[0] = DFS747_REG_DET_CDS_CTL_0;
            addr[1] = DFS747_REG_DET_CDS_CTL_1;
            addr[2] = DFS747_REG_DET_PGA0_CTL;
            addr[3] = DFS747_REG_DET_PGA1_CTL;

            data[0] = 0x00;
            data[1] = 0xFF;
            data[2] = 0x00;
            data[3] = 0x02;

            cds_offset = (((uint16_t) (data[0] & 0x80)) << 1) | ((uint16_t) data[1]);

            status = fps_multiple_write(dev_fd, addr, data, 4);
            if (status < 0) {
                goto calibrate_detect4_error;
            }

            // Set Double Sensitivity
            status = fps_single_read(dev_fd, DFS747_REG_DET_PGA0_CTL, &data[0]);
            if (status < 0) {
                goto calibrate_detect4_error;
            }

            if (double_sense > 0) {
                data[0] |=  (1 << 5);
            } else {
                data[0] &= ~(1 << 5);
            }

            status = fps_single_write(dev_fd, DFS747_REG_DET_PGA0_CTL, data[0]);
            if (status < 0) {
                goto calibrate_detect4_error;
            }

            // Set suspend interval
            addr[0] = DFS747_REG_SUSP_WAIT_F_CYC_H;
            addr[1] = DFS747_REG_SUSP_WAIT_F_CYC_L;

            data[0] = (uint8_t) ((det_frame & 0xFF00) >> 8);
            data[1] = (uint8_t) ((det_frame & 0x00FF) >> 0);

            status = fps_multiple_write(dev_fd, addr, data, 2);
            if (status < 0) {
                goto calibrate_detect4_error;
            }

            det_size = (col_end - col_begin + 1) * (row_end - row_begin + 1);

            // Assume the slowest oscillator frequency is 250KHz, i.e. 4us
            sleep_us = (double) (det_size * (1 + det_frame) * 8 * 4) * sleep_mul;

            // Start time measurement
            status = gettimeofday(&start_time, NULL);
            if (status < 0) {
                goto calibrate_detect4_error;
            }

            // Use the background image to search the best detect window
            status = fps_search_detect_window_from_bkgnd_image(col_begin, col_end,
                                                               &row_begin, &row_end,
                                                               NULL, &win_dev);
            if (status < 0) {
                goto calibrate_detect4_error;
            }

            // Set detect line
            addr[0] = DFS747_REG_DET_ROW_BEGIN;
            addr[1] = DFS747_REG_DET_ROW_END;
            addr[2] = DFS747_REG_DET_COL_BEGIN;
            addr[3] = DFS747_REG_DET_COL_END;

            data[0] = row_begin;
            data[1] = row_end;
            data[2] = col_begin;
            data[3] = col_end;

            status = fps_multiple_write(dev_fd, addr, data, 4);
            if (status < 0) {
                goto calibrate_detect4_error;
            }

            // Switch to detect mode
#if 0
            status = fps_switch_mode(dev_fd, DFS747_DETECT_MODE, &mode_old);
            if (status < 0) {
                goto calibrate_detect4_error;
            }
#else
            status = fps_switch_mode(dev_fd, DFS747_POWER_DOWN_MODE, &mode_old);
            status = fps_switch_mode(dev_fd, DFS747_DETECT_MODE, NULL);
            if (status < 0) {
                goto calibrate_detect4_error;
            }
#endif

            for (retry_cnt = 0; retry_cnt < retry_limit; retry_cnt++) {

                det_max = DFS747_MAX_DETECT_TH;
                det_min = DFS747_MIN_DETECT_TH;

                det_upper  = det_max;
                det_lower  = det_min;
                det_middle = (det_upper + det_lower) / 2;

                while ((det_upper - det_lower) > 2) {
                    for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
                        status = fps_scan_detect_event(dev_fd, det_middle, cds_offset,
                                                       sleep_us, int_enable, (scan_cnt == 0));
                        if (status < 0) {
                            goto calibrate_detect4_error;
                        }

                        if ((status > 0) && (scan_cnt > 2)) {
                            break;
                        }
                    }

                    DEBUG("1. Detect Th. = 0x%02X, CDS Offset = 0x%03X, finger_on = %0d, finger_off = %0d\n",
                          det_middle, cds_offset, finger_on, finger_off);

                    if (scan_cnt == scan_limit) {
                        det_upper  = det_middle;
                        det_middle = (det_upper + det_lower) / 2;
                    } else {
                        det_lower  = det_middle;
                        det_middle = (det_upper + det_lower) / 2;
                    }
                }

                if (det_middle >= det_max) {
                    WARN("Maximum Detect Threshold reached! Retry...\n");
                    usleep(sleep_us);
                    continue;
                }
                if (det_middle <= det_min) {
                    WARN("Minimum Detect Threshold reached! Retry...\n");
                    usleep(sleep_us);
                    continue;
                }

                detect_th = det_middle + extra_detect_th;
                if (detect_th > DFS747_MAX_DETECT_TH) {
                    detect_th = DFS747_MAX_DETECT_TH;
                }
                if (detect_th < DFS747_MIN_DETECT_TH) {
                    detect_th = DFS747_MIN_DETECT_TH;
                }

                break;
            }

            if (retry_cnt == retry_limit) {
                DEBUG("Retry time out! Exit...\n");
                goto calibrate_detect4_error;
            }

            if (fine_tune_enable > 0) {
                for (; retry_cnt < retry_limit; retry_cnt++) {

                    cds_max = DFS747_MAX_CDS_OFFSET / 2;
                    cds_min = DFS747_MIN_CDS_OFFSET;

                    cds_upper  = cds_max;
                    cds_lower  = cds_min;
                    cds_middle = (cds_upper + cds_lower) / 2;

#if 0
                    while ((cds_upper - cds_lower) > 2) {
                        finger_on  = 0;
                        finger_off = 0;
                        for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
                            status = fps_scan_detect_event(dev_fd, detect_th, cds_middle,
                                                           sleep_us, int_enable, (scan_cnt == 0));
                            if (status < 0) {
                                goto calibrate_detect4_error;
                            }

                            if (scan_cnt > 0) {
                                if (status > 0) {
                                    finger_on++;
                                } else {
                                    finger_off++;
                                }
                            }
                        }

                        DEBUG("2. Detect Th. = 0x%02X, CDS Offset = 0x%03X, finger_on = %0d, finger_off = %0d\n",
                              detect_th, cds_middle, finger_on, finger_off);

                        if (finger_on < finger_off) {
                            cds_upper  = cds_middle;
                            cds_middle = (cds_upper + cds_lower) / 2;
                        }

                        if (finger_off < finger_on) {
                            cds_lower  = cds_middle;
                            cds_middle = (cds_upper + cds_lower) / 2;
                        }
                    }
#else
                    while ((cds_upper - cds_lower) > 2) {
                        for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
                            status = fps_scan_detect_event(dev_fd, detect_th, cds_middle,
                                                           sleep_us, int_enable, (scan_cnt == 0));
                            if (status < 0) {
                                goto calibrate_detect4_error;
                            }

                            if ((status == 0) && (scan_cnt > 0)) {
                                break;
                            }
                        }

                        DEBUG("2. Detect Th. = 0x%02X, CDS Offset = 0x%03X, finger_on = %0d, finger_off = %0d\n",
                              detect_th, cds_middle, finger_on, finger_off);

                        if (scan_cnt == scan_limit) {
                            cds_lower  = cds_middle;
                            cds_middle = (cds_upper + cds_lower) / 2;
                        } else {
                            cds_upper  = cds_middle;
                            cds_middle = (cds_upper + cds_lower) / 2;
                        }
                    }
#endif

                    if (cds_middle >= cds_max) {
                        WARN("Maximum CDS Offset reached! Retry...\n");
                        usleep(sleep_us);
                        continue;
                    }
                    if (cds_middle <= cds_min) {
                        WARN("Minimum CDS Offset reached! Retry...\n");
                        usleep(sleep_us);
                        continue;
                    }

                    cds_offset = cds_middle + extra_cds_offset;
                    if (cds_offset > DFS747_MAX_CDS_OFFSET) {
                        cds_offset = DFS747_MAX_CDS_OFFSET;
                    }
                    if (cds_offset < DFS747_MIN_CDS_OFFSET) {
                        cds_offset = DFS747_MIN_CDS_OFFSET;
                    }

                    break;
                }

                if (retry_cnt == retry_limit) {
                    DEBUG("Retry time out! Exit...\n");
                    goto calibrate_detect4_error;
                }
            }

            // Fill the result to the registers
            addr[0] = DFS747_REG_DET_CDS_CTL_0;
            addr[1] = DFS747_REG_DET_CDS_CTL_1;
            addr[2] = DFS747_REG_V_DET_SEL;

            status = fps_multiple_read(dev_fd, addr, data, 3);
            if (status < 0) {
                goto calibrate_detect4_error;
            }

            data[0] = (data[0] & 0x7F) | ((uint8_t) ((cds_offset & 0x0100) >> 1));
            data[1] = (uint8_t) (cds_offset & 0x00FF);
            data[2] = detect_th;

            status = fps_multiple_write(dev_fd, addr, data, 3);
            if (status < 0) {
                goto calibrate_detect4_error;
            }

            // Stop time measurement
            status = gettimeofday(&stop_time, NULL);
            if (status < 0) {
                goto calibrate_detect4_error;
            }

            elapsed = get_elapsed_ms(&start_time, &stop_time);
            if (elapsed < 0) {
                goto calibrate_detect4_error;
            }

            printf("    Done!\n");
            printf("\n");
            printf("    Selected Row (Begin/End) = %0d/%0d\n", row_begin, row_end);
            printf("    Detect Threshold         = 0x%02X\n",  detect_th);
            printf("    CDS Offset               = 0x%03X\n",  cds_offset);
            printf("\n");
            printf("    Time elapsed = %0.3f ms\n", elapsed);
        }

        printf("\n");
        printf("Press ENTER key to continue... ");
        (void) getchar();
    }

    return status;

calibrate_detect4_error :

    printf("\n");
    printf("    Failed!\n");
    printf("\n");

    return status;
}

static void
recalibrate_detect3(int signum)
{
    int       status             = 0;
    uint32_t  scan_limit         = 4;
    uint32_t  scan_cnt           = 0;
    uint8_t   addr[3];
    uint8_t   data[3];
    uint16_t  cds_offset         = DFS747_MAX_CDS_OFFSET;
    uint8_t   detect_th          = DFS747_MAX_DETECT_TH;
    double    sleep_us           = 0.0;
    uint32_t  score_1st          = 0;
    uint32_t  score_2nd          = 0;
    uint16_t  extra_cds_offset   = 0;
    int       too_insensitive    = 0;
    int       too_sensitive      = 0;
    time_t    raw_time;
    struct tm *time_info;
    char      time_str[80];

    // Load current settings
    detect_th        = det_detect_th;
    cds_offset       = det_cds_offset;
    sleep_us         = det_sleep_us;
    extra_cds_offset = det_extra_cds_offset;

    // A1. we check if the threshold is too high
    score_1st = scan_limit;
    for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
        status = fps_scan_detect_event(dev_fd, (detect_th - 1), cds_offset,
                                       sleep_us, 1, 1);

        // Finger-on is still NOT detected, decr. Detect Th.
        if (status == 0) {
            if (detect_th != DFS747_MIN_DETECT_TH) {
                DEBUG("%s(): detect_th--!\n", __func__);
                detect_th--;
                score_1st--;
            } else {
                DEBUG("%s(): No more settings!\n", __func__);
                goto recalibrate_detect3_error;
            }
        }
    }

    too_insensitive = (score_1st != scan_limit);
    DEBUG("%s(): A1. Score = %0d (%s)\n", __func__, score_1st, (too_insensitive ? "Insensitive" : "OK"));

    // A2. we check if the threshold is too low
    score_2nd = scan_limit;
    for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
        status = fps_scan_detect_event(dev_fd, (detect_th + 1), cds_offset,
                                       sleep_us, 1, 1);

        // Finger-on is still detected, incr. Detect Th.
        if (status > 0) {
            if (detect_th != DFS747_MAX_DETECT_TH) {
                DEBUG("%s(): detect_th++!\n", __func__);
                detect_th++;
                score_2nd--;
            } else {
                DEBUG("%s(): No more settings!\n", __func__);
                goto recalibrate_detect3_error;
            }
        }
    }

    too_sensitive = (score_2nd != scan_limit);
    DEBUG("%s(): A2. Score = %0d (%s)\n", __func__, score_2nd, (too_sensitive ? "Sensitive" : "OK"));

    // If the detect th changed, we need to search CDS offset again.
    if (detect_th != det_detect_th) {
        detect_th++;

        status = fps_search_cds_offset(dev_fd, detect_th,
                                       DFS747_MAX_CDS_OFFSET, DFS747_MIN_CDS_OFFSET,
                                       sleep_us, scan_limit, 1,
                                       &cds_offset);
        if (status < 0) {
            goto recalibrate_detect3_error;
        }
        DEBUG("%s(): Detect Th. = 0x%02X, CDS Offset = 0x%03X\n", __func__, detect_th, cds_offset);
    }

    // B1. we check if the threshold is too high
    score_1st = scan_limit;
    for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
        status = fps_scan_detect_event(dev_fd, detect_th, (cds_offset - extra_cds_offset),
                                       sleep_us, 1, 1);

        // Finger-on is still NOT detected, decr. CDS offset
        if (status == 0) {
            if (cds_offset != DFS747_MIN_CDS_OFFSET) {
                DEBUG("%s(): cds_offset--!\n", __func__);
                cds_offset--;
                score_1st--;
            } else {
                DEBUG("%s(): No more settings!\n", __func__);
                goto recalibrate_detect3_error;
            }
        }
    }

    too_insensitive = (score_1st != scan_limit);
    DEBUG("%s(): B1. Score = %0d (%s)\n", __func__, score_1st, (too_insensitive ? "Insensitive" : "OK"));

    // B2. we check if the threshold is too low
    score_2nd = scan_limit;
    for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
        status = fps_scan_detect_event(dev_fd, detect_th, (cds_offset + extra_cds_offset),
                                       sleep_us, 1, 1);

        // Finger-on is still detected, incr. CDS offset
        if (status > 0) {
            if (cds_offset != DFS747_MAX_CDS_OFFSET) {
                DEBUG("%s(): cds_offset++!\n", __func__);
                cds_offset++;
                score_2nd--;
            } else {
                DEBUG("%s(): No more settings!\n", __func__);
                goto recalibrate_detect3_error;
            }
        }
    }

    too_sensitive = (score_2nd != scan_limit);
    DEBUG("%s(): B2. Score = %0d (%s)\n", __func__, score_2nd, (too_sensitive ? "Sensitive" : "OK"));

    // Add some offset to avoid false trigger
    cds_offset += 2;
    if (cds_offset > DFS747_MAX_CDS_OFFSET) {
        cds_offset = DFS747_MAX_CDS_OFFSET;
    }

    // Fill the result to the registers
    addr[0] = DFS747_REG_DET_CDS_CTL_0;
    addr[1] = DFS747_REG_DET_CDS_CTL_1;
    addr[2] = DFS747_REG_V_DET_SEL;

    status = fps_multiple_read(dev_fd, addr, data, 3);
    if (status < 0) {
        goto recalibrate_detect3_error;
    }

    data[0] = (data[0] & 0x7F) | ((uint8_t) ((cds_offset & 0x0100) >> 1));
    data[1] = (uint8_t) (cds_offset & 0x00FF);
    data[2] = detect_th;

    status = fps_multiple_write(dev_fd, addr, data, 3);
    if (status < 0) {
        goto recalibrate_detect3_error;
    }

    time(&raw_time);
    time_info = localtime(&raw_time);
    strftime(time_str, sizeof(time_str), "%F %T", time_info);

    DEBUG("%s(): Detect Th. = 0x%02X, CDS Offset = 0x%03X\n", __func__, detect_th, cds_offset);
    DEBUG("%s(): Current Interrupt Count = %0d (%s)\n", __func__, (detect_cnt - 1), time_str);

    det_detect_th  = detect_th;
    det_cds_offset = cds_offset;

    recal_done = 1;
    alarm(recal_secs);

    return /* 0 */;

recalibrate_detect3_error:

    return /* -1 */;
}

int
calibrate_detect3()
{
    int            status           = 0;
    char           key              = 0;
    char           line[CHAR_CNT];
    char           *arg             = NULL;
    int            user_input       = 0;
    uint16_t       det_frame        = 4;
    uint32_t       scan_limit       = 20;
    uint32_t       scan_cnt         = 0;
    uint8_t        addr[4];
    uint8_t        data[4];
    int            mode_old         = 0;
    double         win_dev          = 0.0;
    uint8_t        row_begin        = (DFS747_SENSOR_ROWS / 2) - 4;
    uint8_t        row_end          = (DFS747_SENSOR_ROWS / 2) + 3;
    uint8_t        col_begin        = (DFS747_SENSOR_COLS / 2) - 4;
    uint8_t        col_end          = (DFS747_SENSOR_COLS / 2) + 3;
    uint32_t       det_size         = 8;
    uint16_t       cds_offset       = DFS747_MAX_CDS_OFFSET;
    uint8_t        detect_th        = DFS747_MAX_DETECT_TH;
    double         sleep_us         = 0.0;
    struct timeval start_time;
    struct timeval stop_time;
    double         elapsed          = 0.0;
    uint16_t       extra_cds_offset = 0;
    int            int_enable       = 1;
    int            fine_tune_enable = 1;
    int            cal_success      = 1;
    uint32_t       repeat_times     = 8;
    uint32_t       score_1st        = 0;
    uint32_t       score_2nd        = 0;
    int            too_insensitive  = 0;
    int            too_sensitive    = 0;
    uint32_t       retry_cnt        = 0;
    int            double_sense     = 0;
    int            detect_th_check  = 0;
    int            cds_offset_check = 1;

    while (1) {
        clear_console();

        printf("\n");
        printf("=============================\n");
        printf(" Calibrate Detect (Method 3) \n");
        printf("=============================\n");
        printf("\n");
        printf("    's' frm - Specify suspend interval.                       \n");
        printf("                frm = Suspend interval, and uint is frames.   \n");
        printf("              This number must be decimal.                    \n");
        printf("    't' scn - Specify detect scan times in a iteration.       \n");
        printf("                scn = Detect scan times.                      \n");
        printf("              This number must be decimal.                    \n");
        printf("    'f' fn  - Enable fine-tune stage.                         \n");
        printf("                fn = Enable or disable fine-tune stage.       \n");
        printf("                  0: Disable.                                 \n");
        printf("                  1: Enable.                                  \n");
        printf("              This number must be decimal.                    \n");
        printf("    'i' ie  - Enable detect interrupt.                        \n");
        printf("                ie = Enable or disable detect interrupt.      \n");
        printf("                  0: Disable.                                 \n");
        printf("                  1: Enable.                                  \n");
        printf("              This number must be decimal.                    \n");
        printf("    'd' db  - Enable double sensitivity circuit.              \n");
        printf("                db = Enable or disable double sensitivity.    \n");
        printf("                  0: Disable.                                 \n");
        printf("                  1: Enable.                                  \n");
        printf("              This number must be decimal.                    \n");
        printf("    'g'     - Get current detect calibration settings.        \n");
        printf("    ENTER   - Do detect calibration based on current settings.\n");
        printf("    'q'     - Back to main menu.                              \n");
        printf("\n");
        printf("Pleae enter: ");

        clear_line(line, sizeof(line));
        fgets(line, sizeof(line), stdin);
        key = line[0];

        if (key == 'q') {
            break;
        }

        printf("\n");
        printf("Result:\n");
        printf("\n");

        if (key == 's') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 1) {
                det_frame = (uint16_t) user_input;
                goto calibrate_detect3_show_options;
            } else {
                printf("    ERROR: Suspend interval must be >= 1!\n");
            }
        }

        if (key == 't') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 1) {
                scan_limit = (uint32_t) user_input;
                goto calibrate_detect3_show_options;
            } else {
                printf("    ERROR: Scan limit must be >= 1!\n");
            }
        }

        if (key == 'f') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 0) {
                fine_tune_enable = user_input;
                goto calibrate_detect3_show_options;
            } else {
                printf("    ERROR: This option must be either 0 or 1!\n");
            }
        }

        if (key == 'i') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 0) {
                int_enable = user_input;
                goto calibrate_detect3_show_options;
            } else {
                printf("    ERROR: This option must be either 0 or 1!\n");
            }
        }

        if (key == 'd') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 0) {
                double_sense = user_input;
                goto calibrate_detect3_show_options;
            } else {
                printf("    ERROR: This option must be either 0 or 1!\n");
            }
        }

        if (key == 'g') {

calibrate_detect3_show_options :

            printf("    Suspend Interval          = %0d frames\n", det_frame);
            printf("    Scan Limit                = %0d\n",        scan_limit);
            printf("    Enable Fine Tune Stage    = %s\n",         (fine_tune_enable > 0) ? "TRUE" : "FALSE");
            printf("    Enable Detect Interrupt   = %s\n",         (int_enable       > 0) ? "TRUE" : "FALSE");
            printf("    Enable Double Sensitivity = %s\n",         (double_sense     > 0) ? "TRUE" : "FALSE");
        }

        if (key == '\n') {
            printf("    Calibrating...\n");

            // Disable all interrupts
            status = fps_single_write(dev_fd, DFS747_REG_INT_CTL, 0x00);
            if (status < 0) {
                goto calibrate_detect3_error;
            }

            // Set Detect Mode PGA gain
#if 0
            status = fps_single_write(dev_fd, DFS747_REG_DET_PGA1_CTL, 0x01);
#else
            status = fps_single_write(dev_fd, DFS747_REG_DET_PGA1_CTL, 0x0F);
#endif
            if (status < 0) {
                goto calibrate_detect3_error;
            }

            // Set Double Sensitivity
            status = fps_single_read(dev_fd, DFS747_REG_DET_PGA0_CTL, &data[0]);
            if (status < 0) {
                goto calibrate_detect3_error;
            }

            if (double_sense > 0) {
                data[0] |=  (1 << 5);
            } else {
                data[0] &= ~(1 << 5);
            }

            status = fps_single_write(dev_fd, DFS747_REG_DET_PGA0_CTL, data[0]);
            if (status < 0) {
                goto calibrate_detect3_error;
            }

            // Set suspend interval
            addr[0] = DFS747_REG_SUSP_WAIT_F_CYC_H;
            addr[1] = DFS747_REG_SUSP_WAIT_F_CYC_L;

            data[0] = (uint8_t) ((det_frame & 0xFF00) >> 8);
            data[1] = (uint8_t) ((det_frame & 0x00FF) >> 0);

            status = fps_multiple_write(dev_fd, addr, data, 2);
            if (status < 0) {
                goto calibrate_detect3_error;
            }

            det_size = (col_end - col_begin + 1) * (row_end - row_begin + 1);

            sleep_us = (double) (det_size * (1 + det_frame) * 10 * 8 * 2500) / 1000;
            if (sleep_us < 20.0 * MSEC) {
                sleep_us = 20.0 * MSEC;
            }

calibrate_detect3_retry :

            // Start time measurement
            status = gettimeofday(&start_time, NULL);
            if (status < 0) {
                goto calibrate_detect3_error;
            }

            // Switch to image mode to search the best detect window
            status = fps_switch_mode(dev_fd, DFS747_IMAGE_MODE, &mode_old);
            if (status < 0) {
                goto calibrate_detect3_error;
            }

            // Search the best detect window
            status = fps_search_detect_window(dev_fd, col_begin, col_end,
                                              repeat_times, &extra_cds_offset,
                                              &row_begin, &row_end, &win_dev);
            if (status < 0) {
                goto calibrate_detect3_error;
            }

            // Set detect line
            addr[0] = DFS747_REG_DET_ROW_BEGIN;
            addr[1] = DFS747_REG_DET_ROW_END;
            addr[2] = DFS747_REG_DET_COL_BEGIN;
            addr[3] = DFS747_REG_DET_COL_END;

            data[0] = row_begin;
            data[1] = row_end;
            data[2] = col_begin;
            data[3] = col_end;

            status = fps_multiple_write(dev_fd, addr, data, 4);
            if (status < 0) {
                goto calibrate_detect3_error;
            }

            // Switch to detect mode
            status = fps_switch_mode(dev_fd, DFS747_DETECT_MODE, NULL);
            if (status < 0) {
                goto calibrate_detect3_error;
            }

            cds_offset = DFS747_MAX_CDS_OFFSET;

            // Skip first scan
            status = fps_scan_detect_event(dev_fd, detect_th, cds_offset, sleep_us, int_enable, 1);
            if (status < 0) {
                goto calibrate_detect3_error;
            }

calibrate_detect3_search_detect_th_retry:

            // Search Detect Th.
            status = fps_search_detect_threshold(dev_fd, cds_offset,
                                                 DFS747_MAX_DETECT_TH, DFS747_MIN_DETECT_TH,
                                                 sleep_us, 1, int_enable,
                                                 &detect_th);
            if (status < 0) {
                goto calibrate_detect3_error;
            }
            DEBUG("%s(): Detect Th. = 0x%02X, CDS Offset = 0x%03X\n", __func__, detect_th, cds_offset);

            if ((detect_th >= (DFS747_MAX_DETECT_TH - 0x0F)) ||
                (detect_th <= 0x0F)) {
                DEBUG("%s(): Search Detect Th. again...\n", __func__);
                goto calibrate_detect3_search_detect_th_retry;
            }

            if (detect_th_check > 0) {
                // A1. we check if the threshold is too high
                score_1st = scan_limit;
                for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
                    status = fps_scan_detect_event(dev_fd, (detect_th - 1), cds_offset,
                                                   sleep_us, int_enable, 1);

                    // Finger-on is still NOT detected, decr. Detect Th.
                    if (status == 0) {
                        if (detect_th != DFS747_MIN_DETECT_TH) {
                            DEBUG("%s(): detect_th--!\n", __func__);
                            detect_th--;
                            score_1st--;
                        } else {
                            DEBUG("%s(): No more settings!\n", __func__);
                            cal_success = 0;
                            break;
                        }
                    }
                }

                too_insensitive = (score_1st != scan_limit);
                DEBUG("%s(): A1. Score = %0d (%s)\n", __func__, score_1st, (too_insensitive ? "Insensitive" : "OK"));

                // A2. we check if the threshold is too low
                score_2nd = scan_limit;
                for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
                    status = fps_scan_detect_event(dev_fd, (detect_th + 1), cds_offset,
                                                   sleep_us, int_enable, 1);

                    // Finger-on is still detected, incr. Detect Th.
                    if (status > 0) {
                        if (detect_th != DFS747_MAX_DETECT_TH) {
                            DEBUG("%s(): detect_th++!\n", __func__);
                            detect_th++;
                            score_2nd--;
                        } else {
                            DEBUG("%s(): No more settings!\n", __func__);
                            cal_success = 0;
                            break;
                        }
                    }
                }

                too_sensitive = (score_2nd != scan_limit);
                DEBUG("%s(): A2. Score = %0d (%s)\n", __func__, score_2nd, (too_sensitive ? "Sensitive" : "OK"));
            }

            if (fine_tune_enable > 0) {

calibrate_detect3_search_cds_offset_retry:

                // Search CDS Offset
                status = fps_search_cds_offset(dev_fd, detect_th,
                                               DFS747_MAX_CDS_OFFSET, DFS747_MIN_CDS_OFFSET,
                                               sleep_us, scan_limit, int_enable,
                                               &cds_offset);
                if (status < 0) {
                    goto calibrate_detect3_error;
                }
                DEBUG("%s(): Detect Th. = 0x%02X, CDS Offset = 0x%03X\n", __func__, detect_th, cds_offset);

                if (cds_offset >= 0x1F0) {
                    DEBUG("%s(): Search CDS Offset again...\n", __func__);
                    goto calibrate_detect3_search_cds_offset_retry;
                }

                if (cds_offset_check > 0) {
                    // B1. we check if the threshold is too high
                    score_1st = scan_limit;
                    for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
                        status = fps_scan_detect_event(dev_fd, detect_th, (cds_offset - extra_cds_offset),
                                                       sleep_us, int_enable, 1);

                        // Finger-on is still NOT detected, decr. CDS offset
                        if (status == 0) {
                            if (cds_offset != DFS747_MIN_CDS_OFFSET) {
                                DEBUG("%s(): cds_offset--!\n", __func__);
                                cds_offset--;
                                score_1st--;
                            } else {
                                DEBUG("%s(): No more settings!\n", __func__);
                                cal_success = 0;
                                break;
                            }
                        }
                    }

                    too_insensitive = (score_1st != scan_limit);
                    DEBUG("%s(): B1. Score = %0d (%s)\n", __func__, score_1st, (too_insensitive ? "Insensitive" : "OK"));

                    // B2. we check if the threshold is too low
                    score_2nd = scan_limit;
                    for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
                        status = fps_scan_detect_event(dev_fd, detect_th, (cds_offset + extra_cds_offset),
                                                       sleep_us, int_enable, 1);

                        // Finger-on is still detected, incr. CDS offset
                        if (status > 0) {
                            if (cds_offset != DFS747_MAX_CDS_OFFSET) {
                                DEBUG("%s(): cds_offset++!\n", __func__);
                                cds_offset++;
                                score_2nd--;
                            } else {
                                DEBUG("%s(): No more settings!\n", __func__);
                                cal_success = 0;
                                break;
                            }
                        }
                    }

                    too_sensitive = (score_2nd != scan_limit);
                    DEBUG("%s(): B2. Score = %0d (%s)\n", __func__, score_2nd, (too_sensitive ? "Sensitive" : "OK"));
                }

                // Add some offset to avoid false trigger
                cds_offset += 2;
                if (cds_offset > DFS747_MAX_CDS_OFFSET) {
                    cds_offset = DFS747_MAX_CDS_OFFSET;
                }
            }

            // Fill the result to the registers
            addr[0] = DFS747_REG_DET_CDS_CTL_0;
            addr[1] = DFS747_REG_DET_CDS_CTL_1;
            addr[2] = DFS747_REG_V_DET_SEL;

            status = fps_multiple_read(dev_fd, addr, data, 3);
            if (status < 0) {
                goto calibrate_detect3_error;
            }

            data[0] = (data[0] & 0x7F) | ((uint8_t) ((cds_offset & 0x0100) >> 1));
            data[1] = (uint8_t) (cds_offset & 0x00FF);
            data[2] = detect_th;

            status = fps_multiple_write(dev_fd, addr, data, 3);
            if (status < 0) {
                goto calibrate_detect3_error;
            }

            // Switch back to original mode
            status = fps_switch_mode(dev_fd, mode_old, &mode_old);
            if (status < 0) {
                goto calibrate_detect3_error;
            }

            // Stop time measurement
            status = gettimeofday(&stop_time, NULL);
            if (status < 0) {
                goto calibrate_detect3_error;
            }

            elapsed = get_elapsed_ms(&start_time, &stop_time);
            if (elapsed < 0) {
                goto calibrate_detect3_error;
            }

#if 0
            if (((too_insensitive > 0) && (too_sensitive > 0)) ||
                (score_1st < (scan_limit - 4)) ||
                (score_2nd < (scan_limit - 4))) {
                if (retry_cnt == 3) {
                    DEBUG("%s(): Enough! Stop...\n", __func__);
                    goto calibrate_detect3_error;
                }
                WARN("%s(): Weird! Try again...\n", __func__);
                retry_cnt++;
                goto calibrate_detect3_retry;
            }
#endif

            if (cal_success > 0) {
                printf("    Done!\n");
                printf("\n");
                printf("    Selected Row (Begin/End) = %0d/%0d\n", row_begin, row_end);
                printf("    Detect Threshold         = 0x%02X\n",  detect_th);
                printf("    CDS Offset               = 0x%03X\n",  cds_offset);
                printf("\n");
                printf("    Time elapsed = %0.3f ms\n", elapsed);

                det_detect_th        = detect_th;
                det_cds_offset       = cds_offset;
                det_sleep_us         = sleep_us;
                det_extra_cds_offset = extra_cds_offset;
            } else {
                printf("    Failed!\n");
            }
        }

        printf("\n");
        printf("Press ENTER key to continue... ");
        (void) getchar();
    }

    return status;

calibrate_detect3_error :

    printf("\n");
    printf("    Failed!\n");
    printf("\n");

    return status;
}

int
calibrate_detect2()
{
    int            status           = 0;
    char           key              = 0;
    char           line[CHAR_CNT];
    char           *arg             = NULL;
    int            user_input       = 0;
    uint16_t       det_frame        = 2;
    uint32_t       scan_limit       = 10;
    uint32_t       scan_cnt         = 0;
    uint8_t        addr[4];
    uint8_t        data[4];
    int            mode_old         = 0;
    uint8_t        row_found        = DFS747_SENSOR_ROWS / 2;
    double         row_dev          = 0.0;
    uint8_t        col_begin        = (DFS747_SENSOR_COLS / 2) - 8;
    uint8_t        col_end          = (DFS747_SENSOR_COLS / 2) + 7;
    uint32_t       det_width        = 8;
    uint16_t       cds_offset       = DFS747_MAX_CDS_OFFSET;
    uint8_t        detect_th        = DFS747_MAX_DETECT_TH;
    double         sleep_us         = 0.0;
    struct timeval start_time;
    struct timeval stop_time;
    double         elapsed          = 0.0;
    uint16_t       extra_cds_offset = 0;
    int            int_enable       = 1;
    int            fine_tune_enable = 1;
    int            cal_success      = 1;
    uint32_t       repeat_times     = 8;
    uint32_t       score_1st        = 0;
    uint32_t       score_2nd        = 0;
    int            too_insensitive  = 0;
    int            too_sensitive    = 0;
    uint32_t       retry_cnt        = 0;

    while (1) {
        clear_console();

        printf("\n");
        printf("=============================\n");
        printf(" Calibrate Detect (Method 2) \n");
        printf("=============================\n");
        printf("\n");
        printf("    's' frm - Specify suspend interval.                       \n");
        printf("                frm = Suspend interval, and uint is frames.   \n");
        printf("              This number must be decimal.                    \n");
        printf("    't' scn - Specify detect scan times in a iteration.       \n");
        printf("                scn = Detect scan times.                      \n");
        printf("              This number must be decimal.                    \n");
        printf("    'f' fn  - Enable fine-tune stage.                         \n");
        printf("                fn = Enable or disable fine-tune stage.       \n");
        printf("                  0: Disable.                                 \n");
        printf("                  1: Enable.                                  \n");
        printf("              This number must be decimal.                    \n");
        printf("    'i' ie  - Enable detect interrupt.                        \n");
        printf("                ie = Enable or disable detect interrupt.      \n");
        printf("                  0: Disable.                                 \n");
        printf("                  1: Enable.                                  \n");
        printf("              This number must be decimal.                    \n");
        printf("    'g'     - Get current detect calibration settings.        \n");
        printf("    ENTER   - Do detect calibration based on current settings.\n");
        printf("    'q'     - Back to main menu.                              \n");
        printf("\n");
        printf("Pleae enter: ");

        clear_line(line, sizeof(line));
        fgets(line, sizeof(line), stdin);
        key = line[0];

        if (key == 'q') {
            break;
        }

        printf("\n");
        printf("Result:\n");
        printf("\n");

        if (key == 's') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 1) {
                det_frame = (uint16_t) user_input;
                goto calibrate_detect2_show_options;
            } else {
                printf("    ERROR: Suspend interval must be >= 1!\n");
            }
        }

        if (key == 't') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 1) {
                scan_limit = (uint32_t) user_input;
                goto calibrate_detect2_show_options;
            } else {
                printf("    ERROR: Scan limit must be >= 1!\n");
            }
        }

        if (key == 'f') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 0) {
                fine_tune_enable = user_input;
                goto calibrate_detect2_show_options;
            } else {
                printf("    ERROR: This option must be either 0 or 1!\n");
            }
        }

        if (key == 'i') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 0) {
                int_enable = user_input;
                goto calibrate_detect2_show_options;
            } else {
                printf("    ERROR: This option must be either 0 or 1!\n");
            }
        }

        if (key == 'g') {

calibrate_detect2_show_options :

            printf("    Suspend Interval        = %0d frames\n", det_frame);
            printf("    Scan Limit              = %0d\n",        scan_limit);
            printf("    Enable Fine Tune Stage  = %s\n",         (fine_tune_enable > 0) ? "TRUE" : "FALSE");
            printf("    Enable Detect Interrupt = %s\n",         (int_enable       > 0) ? "TRUE" : "FALSE");
        }

        if (key == '\n') {
            printf("    Calibrating...\n");

            // Disable all interrupts
            status = fps_single_write(dev_fd, DFS747_REG_INT_CTL, 0x00);
            if (status < 0) {
                goto calibrate_detect2_error;
            }

            // Set Detect Mode PGA gain
            status = fps_single_write(dev_fd, DFS747_REG_DET_PGA1_CTL, 0x01);
            if (status < 0) {
                goto calibrate_detect2_error;
            }

            // Set suspend interval
            addr[0] = DFS747_REG_SUSP_WAIT_F_CYC_H;
            addr[1] = DFS747_REG_SUSP_WAIT_F_CYC_L;

            data[0] = (uint8_t) ((det_frame & 0xFF00) >> 8);
            data[1] = (uint8_t) ((det_frame & 0x00FF) >> 0);

            status = fps_multiple_write(dev_fd, addr, data, 2);
            if (status < 0) {
                goto calibrate_detect2_error;
            }

            det_width = col_end - col_begin + 1;

            sleep_us = (double) (det_width * (1 + det_frame) * 10 * 8 * 2500) / 1000;
            if (sleep_us < 20.0 * MSEC) {
                sleep_us = 20.0 * MSEC;
            }

calibrate_detect2_retry :

            // Start time measurement
            status = gettimeofday(&start_time, NULL);
            if (status < 0) {
                goto calibrate_detect2_error;
            }

            // Switch to image mode to search the best detect window
            status = fps_switch_mode(dev_fd, DFS747_IMAGE_MODE, &mode_old);
            if (status < 0) {
                goto calibrate_detect2_error;
            }

            // Search the best detect window
            status = fps_search_detect_line(dev_fd, col_begin, col_end,
                                            ((DFS747_SENSOR_ROWS / 2) - 4),
                                            ((DFS747_SENSOR_ROWS / 2) + 4),
                                            repeat_times, &extra_cds_offset,
                                            &row_found, &row_dev);
            if (status < 0) {
                goto calibrate_detect2_error;
            }

            // Set detect line
            addr[0] = DFS747_REG_DET_ROW_BEGIN;
            addr[1] = DFS747_REG_DET_ROW_END;
            addr[2] = DFS747_REG_DET_COL_BEGIN;
            addr[3] = DFS747_REG_DET_COL_END;

            data[0] = row_found;
            data[1] = row_found;
            data[2] = col_begin;
            data[3] = col_end;

            status = fps_multiple_write(dev_fd, addr, data, 4);
            if (status < 0) {
                goto calibrate_detect2_error;
            }

            // Switch to detect mode
            status = fps_switch_mode(dev_fd, DFS747_DETECT_MODE, NULL);
            if (status < 0) {
                goto calibrate_detect2_error;
            }

            cds_offset = DFS747_MAX_CDS_OFFSET;

            // Skip first scan
            status = fps_scan_detect_event(dev_fd, detect_th, cds_offset, sleep_us, int_enable, 1);
            if (status < 0) {
                goto calibrate_detect2_error;
            }

            // Search Detect Th.
            status = fps_search_detect_threshold(dev_fd, cds_offset,
                                                 DFS747_MAX_DETECT_TH, DFS747_MIN_DETECT_TH,
                                                 sleep_us, 1, int_enable,
                                                 &detect_th);
            if (status < 0) {
                goto calibrate_detect2_error;
            }
            DEBUG("%s(): Detect Th. = 0x%02X, CDS Offset = 0x%03X\n", __func__, detect_th, cds_offset);

            // A1. we check if the threshold is too high
            score_1st = scan_limit;
            for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
                status = fps_scan_detect_event(dev_fd, (detect_th - 1), cds_offset,
                                               sleep_us, int_enable, 1);

                // Finger-on is still NOT detected, decr. Detect Th.
                if (status == 0) {
                    if (detect_th != DFS747_MIN_DETECT_TH) {
                        DEBUG("%s(): detect_th--!\n", __func__);
                        detect_th--;
                        score_1st--;
                    } else {
                        DEBUG("%s(): No more settings!\n", __func__);
                        cal_success = 0;
                        break;
                    }
                }
            }

            too_insensitive = (score_1st != scan_limit);
            DEBUG("%s(): A1. Score = %0d (%s)\n", __func__, score_1st, (too_insensitive ? "Insensitive" : "OK"));

            // A2. we check if the threshold is too low
            score_2nd = scan_limit;
            for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
                status = fps_scan_detect_event(dev_fd, (detect_th + 1), cds_offset,
                                               sleep_us, int_enable, 1);

                // Finger-on is still detected, incr. Detect Th.
                if (status > 0) {
                    if (detect_th != DFS747_MAX_DETECT_TH) {
                        DEBUG("%s(): detect_th++!\n", __func__);
                        detect_th++;
                        score_2nd--;
                    } else {
                        DEBUG("%s(): No more settings!\n", __func__);
                        cal_success = 0;
                        break;
                    }
                }
            }

            too_sensitive = (score_2nd != scan_limit);
            DEBUG("%s(): A2. Score = %0d (%s)\n", __func__, score_2nd, (too_sensitive ? "Sensitive" : "OK"));

            if (fine_tune_enable > 0) {
                detect_th++;

                // Search CDS Offset
                status = fps_search_cds_offset(dev_fd, detect_th,
                                               DFS747_MAX_CDS_OFFSET, DFS747_MIN_CDS_OFFSET,
                                               sleep_us, scan_limit, int_enable,
                                               &cds_offset);
                if (status < 0) {
                    goto calibrate_detect2_error;
                }
                DEBUG("%s(): Detect Th. = 0x%02X, CDS Offset = 0x%03X\n", __func__, detect_th, cds_offset);

                // B1. we check if the threshold is too high
                score_1st = scan_limit;
                for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
                    status = fps_scan_detect_event(dev_fd, detect_th, (cds_offset - extra_cds_offset),
                                                   sleep_us, int_enable, 1);

                    // Finger-on is still NOT detected, decr. CDS offset
                    if (status == 0) {
                        if (cds_offset != DFS747_MIN_CDS_OFFSET) {
                            DEBUG("%s(): cds_offset--!\n", __func__);
                            cds_offset--;
                            score_1st--;
                        } else {
                            DEBUG("%s(): No more settings!\n", __func__);
                            cal_success = 0;
                            break;
                        }
                    }
                }

                too_insensitive = (score_1st != scan_limit);
                DEBUG("%s(): B1. Score = %0d (%s)\n", __func__, score_1st, (too_insensitive ? "Insensitive" : "OK"));

                // B2. we check if the threshold is too low
                score_2nd = scan_limit;
                for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
                    status = fps_scan_detect_event(dev_fd, detect_th, (cds_offset + extra_cds_offset),
                                                   sleep_us, int_enable, 1);

                    // Finger-on is still detected, incr. CDS offset
                    if (status > 0) {
                        if (cds_offset != DFS747_MAX_CDS_OFFSET) {
                            DEBUG("%s(): cds_offset++!\n", __func__);
                            cds_offset++;
                            score_2nd--;
                        } else {
                            DEBUG("%s(): No more settings!\n", __func__);
                            cal_success = 0;
                            break;
                        }
                    }
                }

                too_sensitive = (score_2nd != scan_limit);
                DEBUG("%s(): B2. Score = %0d (%s)\n", __func__, score_2nd, (too_sensitive ? "Sensitive" : "OK"));
            }

            // Add some offset to avoid false trigger
            cds_offset += 2;
            if (cds_offset > DFS747_MAX_CDS_OFFSET) {
                cds_offset = DFS747_MAX_CDS_OFFSET;
            }

            // Fill the result to the registers
            addr[0] = DFS747_REG_DET_CDS_CTL_0;
            addr[1] = DFS747_REG_DET_CDS_CTL_1;
            addr[2] = DFS747_REG_V_DET_SEL;

            status = fps_multiple_read(dev_fd, addr, data, 3);
            if (status < 0) {
                goto calibrate_detect2_error;
            }

            data[0] = (data[0] & 0x7F) | ((uint8_t) ((cds_offset & 0x0100) >> 1));
            data[1] = (uint8_t) (cds_offset & 0x00FF);
            data[2] = detect_th;

            status = fps_multiple_write(dev_fd, addr, data, 3);
            if (status < 0) {
                goto calibrate_detect2_error;
            }

            // Switch back to original mode
            status = fps_switch_mode(dev_fd, mode_old, &mode_old);
            if (status < 0) {
                goto calibrate_detect2_error;
            }

            // Stop time measurement
            status = gettimeofday(&stop_time, NULL);
            if (status < 0) {
                goto calibrate_detect2_error;
            }

            elapsed = get_elapsed_ms(&start_time, &stop_time);
            if (elapsed < 0) {
                goto calibrate_detect2_error;
            }

            if (((too_insensitive > 0) && (too_sensitive > 0)) ||
                (score_1st < (scan_limit - 4)) ||
                (score_2nd < (scan_limit - 4))) {
                if (retry_cnt == 3) {
                    DEBUG("%s(): Enough! Stop...\n", __func__);
                    goto calibrate_detect2_error;
                }
                WARN("%s(): Weird! Try again...\n", __func__);
                retry_cnt++;
                goto calibrate_detect2_retry;
            }

            if (cal_success > 0) {
                printf("    Done!\n");
                printf("\n");
                printf("    Selected Row     = %0d\n",    row_found);
                printf("    Detect Threshold = 0x%02X\n", detect_th);
                printf("    CDS Offset       = 0x%03X\n", cds_offset);
                printf("\n");
                printf("    Time elapsed = %0.3f ms\n", elapsed);
            } else {
                printf("    Failed!\n");
            }

        }

        printf("\n");
        printf("Press ENTER key to continue... ");
        (void) getchar();
    }

    return status;

calibrate_detect2_error :

    printf("\n");
    printf("    Failed!\n");
    printf("\n");

    return status;
}

int
calibrate_detect1()
{
    int            status               = 0;
    char           key                  = 0;
    char           line[CHAR_CNT];
    char           *arg                 = NULL;
    int            user_input           = 0;
    uint16_t       det_frame            = 2;
    uint32_t       scan_limit           = 10;
    uint32_t       scan_cnt             = 0;
    uint8_t        addr[4];
    uint8_t        data[4];
    int            mode_old             = 0;
    uint8_t        row_found            = DFS747_SENSOR_ROWS / 2;
    double         row_dev              = 0.0;
    uint8_t        col_begin            = (DFS747_SENSOR_COLS / 2) - 8;
    uint8_t        col_end              = (DFS747_SENSOR_COLS / 2) + 7;
    uint32_t       det_width            = 8;
    uint16_t       cds_offset           = DFS747_MAX_CDS_OFFSET;
    uint8_t        detect_th            = DFS747_MAX_DETECT_TH;
    uint8_t        detect_th_search_end = DFS747_MAX_DETECT_TH;
    uint8_t        *detect_th_array     = NULL;
    double         sleep_us             = 0.0;
    struct timeval start_time;
    struct timeval stop_time;
    double         elapsed              = 0.0;
    uint16_t       extra_cds_offset     = 0;
    int            int_enable           = 1;
    int            fine_tune_enable     = 1;
    int            cal_success          = 0;
    uint32_t       cds_search_cnt       = 0;
    uint32_t       curr_cds_range       = 0;
    uint32_t       max_cds_range        = 0;
    uint32_t       repeat_times         = 8;
    uint32_t       score_1st            = 0;
    uint32_t       score_2nd            = 0;
    int            i;
    int            too_insensitive      = 0;
    int            too_sensitive        = 0;
    uint32_t       retry_cnt            = 0;

    while (1) {
        clear_console();

        printf("\n");
        printf("=============================\n");
        printf(" Calibrate Detect (Method 1) \n");
        printf("=============================\n");
        printf("\n");
        printf("    's' frm - Specify suspend interval.                       \n");
        printf("                frm = Suspend interval, and uint is frames.   \n");
        printf("              This number must be decimal.                    \n");
        printf("    't' scn - Specify detect scan times in a iteration.       \n");
        printf("                scn = Detect scan times.                      \n");
        printf("              This number must be decimal.                    \n");
        printf("    'd' dth - Specify the maximum detect threshold to search. \n");
        printf("                dth = Detect threshold search limit.          \n");
        printf("              This number must be hex.                        \n");
        printf("    'f' fn  - Enable fine-tune stage.                         \n");
        printf("                fn = Enable or disable fine-tune stage.       \n");
        printf("                  0: Disable.                                 \n");
        printf("                  1: Enable.                                  \n");
        printf("              This number must be decimal.                    \n");
        printf("    'i' ie  - Enable detect interrupt.                        \n");
        printf("                ie = Enable or disable detect interrupt.      \n");
        printf("                  0: Disable.                                 \n");
        printf("                  1: Enable.                                  \n");
        printf("              This number must be decimal.                    \n");
        printf("    'g'     - Get current detect calibration settings.        \n");
        printf("    ENTER   - Do detect calibration based on current settings.\n");
        printf("    'q'     - Back to main menu.                              \n");
        printf("\n");
        printf("Pleae enter: ");

        clear_line(line, sizeof(line));
        fgets(line, sizeof(line), stdin);
        key = line[0];

        if (key == 'q') {
            break;
        }

        printf("\n");
        printf("Result:\n");
        printf("\n");

        if (key == 's') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 1) {
                det_frame = (uint16_t) user_input;
                goto calibrate_detect1_show_options;
            } else {
                printf("    ERROR: Suspend interval must be >= 1!\n");
            }
        }

        if (key == 't') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 1) {
                scan_limit = (uint32_t) user_input;
                goto calibrate_detect1_show_options;
            } else {
                printf("    ERROR: Scan limit must be >= 1!\n");
            }
        }

        if (key == 'd') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 16);

            if (user_input >= 0) {
                detect_th_search_end = (uint8_t) user_input;
                goto calibrate_detect1_show_options;
            } else {
                printf("    ERROR: Detect threshold search limit must be > 0!\n");
            }
        }

        if (key == 'f') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 0) {
                fine_tune_enable = user_input;
                goto calibrate_detect1_show_options;
            } else {
                printf("    ERROR: This option must be either 0 or 1!\n");
            }
        }

        if (key == 'i') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 0) {
                int_enable = user_input;
                goto calibrate_detect1_show_options;
            } else {
                printf("    ERROR: This option must be either 0 or 1!\n");
            }
        }

        if (key == 'g') {

calibrate_detect1_show_options :

            printf("    Suspend Interval        = %0d frames\n", det_frame);
            printf("    Scan Limit              = %0d\n",        scan_limit);
            printf("    Detect Th. Search Limit = 0x%02X\n",     detect_th_search_end);
            printf("    Enable Fine Tune Stage  = %s\n",         (fine_tune_enable > 0) ? "TRUE" : "FALSE");
            printf("    Enable Detect Interrupt = %s\n",         (int_enable       > 0) ? "TRUE" : "FALSE");
        }

        if (key == '\n') {
            printf("    Calibrating...\n");

            // Disable all interrupts
            status = fps_single_write(dev_fd, DFS747_REG_INT_CTL, 0x00);
            if (status < 0) {
                goto calibrate_detect1_error;
            }

            // Set Detect Mode PGA gain
            status = fps_single_write(dev_fd, DFS747_REG_DET_PGA1_CTL, 0x01);
            if (status < 0) {
                goto calibrate_detect1_error;
            }

            // Set suspend interval
            addr[0] = DFS747_REG_SUSP_WAIT_F_CYC_H;
            addr[1] = DFS747_REG_SUSP_WAIT_F_CYC_L;

            data[0] = (uint8_t) ((det_frame & 0xFF00) >> 8);
            data[1] = (uint8_t) ((det_frame & 0x00FF) >> 0);

            status = fps_multiple_write(dev_fd, addr, data, 2);
            if (status < 0) {
                goto calibrate_detect1_error;
            }

            det_width = col_end - col_begin + 1;

#if 0
            sleep_us = (double) (det_width * (1 + det_frame) * 10 * 8 * 2500) / 1000;
            if (sleep_us < 20.0 * MSEC) {
                sleep_us = 20.0 * MSEC;
            }
#else
            // Assume the slowest oscillator frequency is 250KHz, i.e. 4us
            sleep_us = (double) (det_width * (1 + det_frame) * 8 * 4) * 2 /* sleep_mul */;
#endif

calibrate_detect1_retry :

            // Start time measurement
            status = gettimeofday(&start_time, NULL);
            if (status < 0) {
                goto calibrate_detect1_error;
            }

            // Switch to image mode to search the best detect window
            status = fps_switch_mode(dev_fd, DFS747_IMAGE_MODE, &mode_old);
            if (status < 0) {
                goto calibrate_detect1_error;
            }

            // Search the best detect window
            status = fps_search_detect_line(dev_fd, col_begin, col_end,
                                            ((DFS747_SENSOR_ROWS / 2) - 4),
                                            ((DFS747_SENSOR_ROWS / 2) + 4),
                                            repeat_times, &extra_cds_offset,
                                            &row_found, &row_dev);
            if (status < 0) {
                goto calibrate_detect1_error;
            }

            // Allocate the Detect Threshold lookcup table
            cds_search_cnt  = (DFS747_MAX_CDS_OFFSET + 1) / extra_cds_offset;
            detect_th_array = (uint8_t *) malloc(cds_search_cnt);
            if (detect_th_array == NULL) {
                status = -1;
                goto calibrate_detect1_error;
            }

            // Set detect line
            addr[0] = DFS747_REG_DET_ROW_BEGIN;
            addr[1] = DFS747_REG_DET_ROW_END;
            addr[2] = DFS747_REG_DET_COL_BEGIN;
            addr[3] = DFS747_REG_DET_COL_END;

            data[0] = row_found;
            data[1] = row_found;
            data[2] = col_begin;
            data[3] = col_end;

            status = fps_multiple_write(dev_fd, addr, data, 4);
            if (status < 0) {
                goto calibrate_detect1_error;
            }

            // Switch to detect mode
            status = fps_switch_mode(dev_fd, DFS747_DETECT_MODE, NULL);
            if (status < 0) {
                goto calibrate_detect1_error;
            }

            cds_offset = DFS747_MAX_CDS_OFFSET;

            // Skip first scan
            status = fps_scan_detect_event(dev_fd, detect_th, cds_offset, sleep_us, int_enable, 1);
            if (status < 0) {
                goto calibrate_detect1_error;
            }

            status = fps_search_detect_threshold(dev_fd, cds_offset,
                                                 DFS747_MAX_DETECT_TH, DFS747_MIN_DETECT_TH,
                                                 sleep_us, scan_limit, int_enable,
                                                 &detect_th_array[0]);
            if (status < 0) {
                goto calibrate_detect1_error;
            }
            DEBUG("%s(): [0] Detect Th. = 0x%02X, CDS Offset = 0x%03X\n", __func__, detect_th_array[0], cds_offset);

            cal_success = 1;
            for (i = 1; i < cds_search_cnt; i++) {
                cds_offset = DFS747_MAX_CDS_OFFSET - (i * extra_cds_offset);
                detect_th  = detect_th_array[i - 1];

                while (detect_th <= detect_th_search_end) {
                    status = fps_scan_detect_event(dev_fd, detect_th, cds_offset, sleep_us, int_enable, 1);

                    // Error
                    if (status < 0) {
                        goto calibrate_detect1_error;
                    }

                    // Finger-on is still detected
                    if (status > 0) {
                        detect_th++;
                        DEBUG("%s(): detect_th++!\n", __func__);
                        if (detect_th == DFS747_MAX_DETECT_TH) {
                            cal_success = 0;
                            break;
                        }
                    }

                    if (status == 0) {
                        break;
                    }
                }

                detect_th_array[i] = detect_th;
                DEBUG("%s(): [%0d] Detect Th. = 0x%02X, CDS Offset = 0x%03X\n", __func__, i, detect_th_array[i], cds_offset);
            }

            curr_cds_range = 0;
            max_cds_range  = 0;

            for (i = 0; i < (cds_search_cnt - 1); i++) {
                if (detect_th_array[i] <= detect_th_search_end) {
                    curr_cds_range++;
                    if (detect_th_array[i] != detect_th_array[i + 1]) {
                        if (curr_cds_range > max_cds_range) {
                            max_cds_range = curr_cds_range;
                            cds_offset = DFS747_MAX_CDS_OFFSET - (i * extra_cds_offset);
                            DEBUG("%s(): Detect Th. = 0x%02X, CDS Offset = 0x%03X, Count = %0d\n",
                                  __func__, detect_th_array[i], cds_offset, max_cds_range);
                        }

                        curr_cds_range = 0;
                    }
                }
            }

            // Lookup Detect Threshold from CDS Offset
            detect_th = detect_th_array[(DFS747_MAX_CDS_OFFSET - cds_offset) / extra_cds_offset];

            if (fine_tune_enable > 0) {
                // 1. we check if the threshold is too high
                score_1st = scan_limit;
                for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
                    status = fps_scan_detect_event(dev_fd, detect_th, (cds_offset - extra_cds_offset),
                                                   sleep_us, int_enable, 1);

                    // Finger-on is still NOT detected, decr. CDS offset
                    if (status == 0) {
                        if (cds_offset != DFS747_MIN_CDS_OFFSET) {
                            DEBUG("%s(): cds_offset--!\n", __func__);
                            cds_offset--;
                            score_1st--;
                        } else {
                            DEBUG("%s(): No more settings!\n", __func__);
                            cal_success = 0;
                            break;
                        }
                    }
                }

                too_insensitive = (score_1st != scan_limit);
                DEBUG("%s(): 1. Score = %0d (%s)\n", __func__, score_1st, (too_insensitive ? "Insensitive" : "OK"));

                // 2. we check if the threshold is too low
                score_2nd = scan_limit;
                for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
                    status = fps_scan_detect_event(dev_fd, detect_th, (cds_offset + extra_cds_offset),
                                                   sleep_us, int_enable, 1);

                    // Finger-on is still detected, incr. CDS offset
                    if (status > 0) {
                        if (cds_offset != DFS747_MAX_CDS_OFFSET) {
                            DEBUG("%s(): cds_offset++!\n", __func__);
                            cds_offset++;
                            score_2nd--;
                        } else {
                            DEBUG("%s(): No more settings!\n", __func__);
                            cal_success = 0;
                            break;
                        }
                    }
                }

                too_sensitive = (score_2nd != scan_limit);
                DEBUG("%s(): 2. Score = %0d (%s)\n", __func__, score_2nd, (too_sensitive ? "Sensitive" : "OK"));
            }

            // Add some offset to avoid false trigger
            cds_offset += 2;
            if (cds_offset > DFS747_MAX_CDS_OFFSET) {
                cds_offset = DFS747_MAX_CDS_OFFSET;
            }

            // Fill the result to the registers
            addr[0] = DFS747_REG_DET_CDS_CTL_0;
            addr[1] = DFS747_REG_DET_CDS_CTL_1;
            addr[2] = DFS747_REG_V_DET_SEL;

            status = fps_multiple_read(dev_fd, addr, data, 3);
            if (status < 0) {
                goto calibrate_detect1_error;
            }

            data[0] = (data[0] & 0x7F) | ((uint8_t) ((cds_offset & 0x0100) >> 1));
            data[1] = (uint8_t) (cds_offset & 0x00FF);
            data[2] = detect_th;

            status = fps_multiple_write(dev_fd, addr, data, 3);
            if (status < 0) {
                goto calibrate_detect1_error;
            }

#if 0
            // Switch back to original mode
            status = fps_switch_mode(dev_fd, mode_old, &mode_old);
            if (status < 0) {
                goto calibrate_detect1_error;
            }
#endif

            // Stop time measurement
            status = gettimeofday(&stop_time, NULL);
            if (status < 0) {
                goto calibrate_detect1_error;
            }

            elapsed = get_elapsed_ms(&start_time, &stop_time);
            if (elapsed < 0) {
                goto calibrate_detect1_error;
            }

            if (detect_th_array != NULL) {
                free(detect_th_array);
            }

            if (cal_success > 0) {
                printf("    Done!\n");
                printf("\n");
                printf("    Selected Row     = %0d\n",    row_found);
                printf("    Detect Threshold = 0x%02X\n", detect_th);
                printf("    CDS Offset       = 0x%03X\n", cds_offset);
                printf("\n");
                printf("    Time elapsed = %0.3f ms\n", elapsed);
            } else {
                printf("    Failed!\n");
            }

        }

        printf("\n");
        printf("Press ENTER key to continue... ");
        (void) getchar();
    }

    return status;

calibrate_detect1_error :

    if (detect_th_array != NULL) {
        free(detect_th_array);
    }

    printf("\n");
    printf("    Failed!\n");
    printf("\n");

    return status;
}

int
image_mode_test()
{
    int            status          = 0;
    char           key             = 0;
    char           line[CHAR_CNT];
    char           *arg            = NULL;
    int            user_input_i[2] = {0, 0};
    double         user_input_f    = 0.0;
    uint8_t        avg_frame       = 4;
    uint8_t        num_frame       = 120;
    uint8_t        addr[4];
    uint8_t        data[4];
    int            mode_old        = 0;
    uint8_t        row_begin       = 0;
    uint8_t        row_end         = DFS747_SENSOR_ROWS - 1;
    uint8_t        col_begin       = 0;
    uint8_t        col_end         = DFS747_SENSOR_COLS - 1;
    uint32_t       img_width       = DFS747_SENSOR_COLS;
    uint32_t       img_height      = DFS747_SENSOR_ROWS;
    uint32_t       img_size        = (DFS747_SENSOR_COLS * DFS747_SENSOR_ROWS);
    uint8_t        **raw_buf       = NULL;
    uint8_t        **img_buf       = NULL;
    uint8_t        *avg_img        = NULL;
    double         *sqr_img        = NULL;
    uint8_t        *fng_img        = NULL;
    uint8_t        *enh_img        = NULL;
    uint8_t        *raw_bkg        = NULL;
    double         otsu_mul        = 0.0;
    uint8_t        otsu_th         = 0;
    double         intensity       = 0.0;
    struct timeval start_time;
    struct timeval stop_time;
    double         elapsed         = 0.0;
    time_t         begin_time;
    struct tm      *ptm;
    char           file[CHAR_CNT];
    double         pix             = 0.0;
    double         pix_avg         = 0.0;
    double         pix_sum         = 0.0;
    double         sqr_sum         = 0.0;
    double         bkg_avg         = 0.0;
    double         fng_avg         = 0.0;
    double         fng_dr          = 0.0;
    uint32_t       enh_range       = 0;
    uint8_t        pix_min         = 0;
    uint8_t        pix_max         = 0;
    double         contrast        = 0.0;
    double         brightness      = 0.0;
    int            f;
    int            n;
    int            i;

    while (1) {
        clear_console();

        printf("\n");
        printf("=================\n");
        printf(" Image Mode Test \n");
        printf("=================\n");
        printf("\n");
        printf("    'f' frm   - Specify how many frames to average.             \n");
        printf("                  frm = number of frames to average an image.   \n");
        printf("                This number must be decimal.                    \n");
        printf("    'n' num   - Specify how many images to acquire in this test.\n");
        printf("                  num = Number of image to acquire.             \n");
        printf("                This number must be decimal.                    \n");
        printf("    'o' p1    - Specify the parameter for image enhancement 1.  \n");
        printf("                  p1 = Parameter.                               \n");
        printf("                This number must be floating point.             \n");
        printf("    'e' p2    - Specify the parameter for image enhancement 2.  \n");
        printf("                  p2 = Parameter.                               \n");
        printf("                This number must be decimal.                    \n");
        printf("    'g'       - Get current getting image settings.             \n");
        printf("    'b'       - Get an image and save it as a background.       \n");
        printf("    ENTER     - Get image based on current settings.            \n");
        printf("    'q'       - Back to main menu.                              \n");
        printf("\n");
        printf("Pleae enter: ");

        clear_line(line, sizeof(line));
        fgets(line, sizeof(line), stdin);
        key = line[0];

        if (key == 'q') {
            break;
        }

        printf("\n");
        printf("Result:\n");
        printf("\n");

        if (key == 'f') {
            arg             = strtok(line, " ");
            arg             = strtok(NULL, " ");
            user_input_i[0] = strtol(arg, NULL, 10);

            if (user_input_i[0] >= 1) {
                avg_frame = (uint32_t) user_input_i[0];
                goto image_mode_test_show_options;
            } else {
                printf("    ERROR: Number of frames to average must be >= 1!\n");
            }
        }

        if (key == 'n') {
            arg             = strtok(line, " ");
            arg             = strtok(NULL, " ");
            user_input_i[0] = strtol(arg, NULL, 10);

            if (user_input_i[0] >= 1) {
                num_frame = (uint32_t) user_input_i[0];
                goto image_mode_test_show_options;
            } else {
                printf("    ERROR: Number of frames to acquire must be >= 1!\n");
            }
        }

        if (key == 'o') {
            arg          = strtok(line, " ");
            arg          = strtok(NULL, " ");
            user_input_f = strtod(arg, NULL);

            if (user_input_f >= 0.0) {
                otsu_mul = user_input_f;
                goto image_mode_test_show_options;
            } else {
                printf("    ERROR: Image enhancement 1 parameter must be >= 0 (0 means disabled)!\n");
            }
        }

        if (key == 'e') {
            arg             = strtok(line, " ");
            arg             = strtok(NULL, " ");
            user_input_i[0] = strtol(arg, NULL, 10);

            if (user_input_i[0] >= 0) {
                enh_range = (uint32_t) user_input_i[0];
                goto image_mode_test_show_options;
            } else {
                printf("    ERROR: Image enhancement 2 parameter must be >= 0 (0 means disabled)!\n");
            }
        }

        if (key == 'g') {

image_mode_test_show_options :

            addr[0] = DFS747_REG_IMG_ROW_BEGIN;
            addr[1] = DFS747_REG_IMG_ROW_END;
            addr[2] = DFS747_REG_IMG_COL_BEGIN;
            addr[3] = DFS747_REG_IMG_COL_END;

            status = fps_multiple_read(dev_fd, addr, data, 4);
            if (status < 0) {
                return status;
            }

            row_begin = data[0];
            row_end   = data[1];
            col_begin = data[2];
            col_end   = data[3];

            img_width  = col_end - col_begin + 1;
            img_height = row_end - row_begin + 1;
            img_size   = img_width * img_height;

            printf("    Row Begin    = %0d\n", row_begin);
            printf("    Row End      = %0d\n", row_end);
            printf("    Column Begin = %0d\n", col_begin);
            printf("    COlumn End   = %0d\n", col_end);
            printf("\n");
            printf("    Image Size = %0d (%0dx%0d)\n", img_size, img_width, img_height);
            printf("\n");
            printf("    Number of Frames to Average = %0d\n", avg_frame);
            printf("    Number of Frames to Acquire = %0d\n", num_frame);
            printf("\n");
            printf("    Image Enhancement 1 Parameter  = %0.3f\n", otsu_mul);
            printf("    Image Enhancement 2 Parameter  = %0d\n",   enh_range);
        }

        if (key == 'b') {
            printf("    Getting Background Image...\n");

            raw_bkg = (uint8_t *) malloc(img_size + DFS747_DUMMY_PIXELS);
            if (raw_bkg == NULL) {
                status = -1;
                goto image_mode_test_error;
            }

            // Switch to image mode
#if 0
            status = fps_switch_mode(dev_fd, DFS747_IMAGE_MODE, &mode_old);
            if (status < 0) {
                goto image_mode_test_error;
            }
#else
            status = fps_switch_mode(dev_fd, DFS747_POWER_DOWN_MODE, &mode_old);
            status = fps_switch_mode(dev_fd, DFS747_IMAGE_MODE, NULL);
            if (status < 0) {
                goto image_mode_test_error;
            }
#endif

            // Clear all pending events
            status = fps_single_write(dev_fd, DFS747_REG_INT_EVENT, 0x00);
            if (status < 0) {
                goto image_mode_test_error;
            }

            // Turn on TGEN
            status = fps_enable_tgen(dev_fd, 1);
            if (status < 0) {
                goto image_mode_test_error;
            }

            // Get a frame
            status = fps_get_one_image(dev_fd,
                                       img_width,
                                       img_height,
                                       DFS747_DUMMY_PIXELS,
                                       raw_bkg);
            if (status < 0) {
                goto image_mode_test_error;
            }

            // Turn off TGEN
            status = fps_enable_tgen(dev_fd, 0);
            if (status < 0) {
                goto image_mode_test_error;
            }

            memcpy(bkg_img, &raw_bkg[DFS747_DUMMY_PIXELS], img_size);

            // Free allocated buffer
            if (raw_bkg) free(raw_bkg);

            printf("    Done!\n");
        }

        if (key == '\n') {
            printf("    Getting Images...\n");

            // Create an directory to save images
            time(&begin_time);
            ptm = localtime(&begin_time);
            sprintf(line, "%04d%02d%02d_%02d%02d%02d",
                    (ptm->tm_year + 1900), (ptm->tm_mon + 1), ptm->tm_mday,
                    ptm->tm_hour, ptm->tm_min, ptm->tm_sec);

            status = mkdir(line, 0777);
            if (status < 0) {
                goto image_mode_test_error;
            }

            // Save background image
            sprintf(file, "%s/background.bmp", line);
            status = save_bmp(file, bkg_img, img_size);
            if (status < 0) {
                goto image_mode_test_error;
            }

            // Calculate background image average
            pix_sum = 0.0;
            for (i = 0; i < img_size; i++) {
                pix_sum += (double) bkg_img[i];
            }
            bkg_avg = pix_sum / img_size;

            // Switch to image mode
#if 0
            status = fps_switch_mode(dev_fd, DFS747_IMAGE_MODE, &mode_old);
            if (status < 0) {
                goto image_mode_test_error;
            }
#else
            status = fps_switch_mode(dev_fd, DFS747_POWER_DOWN_MODE, &mode_old);
            status = fps_switch_mode(dev_fd, DFS747_IMAGE_MODE, NULL);
            if (status < 0) {
                goto image_mode_test_error;
            }
#endif

            status = gettimeofday(&start_time, NULL);
            if (status < 0) {
                goto image_mode_test_error;
            }

            // Repeat to aquire and average frames
            for (n = 0; n < num_frame; n++) {
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

                // Create an image buffer to store pixel deviation between frames
                sqr_img = (double *) malloc(img_size * sizeof(double));
                if (sqr_img == NULL) {
                    status = -1;
                    goto image_mode_test_error;
                }

                // Create an image buffer for enhancement
                enh_img = (uint8_t *) malloc(img_size);
                if (enh_img == NULL) {
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
                    status = fps_single_write(dev_fd, DFS747_REG_INT_EVENT, 0x00);
                    if (status < 0) {
                        goto image_mode_test_error;
                    }

                    // Turn on TGEN
                    status = fps_enable_tgen(dev_fd, 1);
                    if (status < 0) {
                        goto image_mode_test_error;
                    }

                    // Get a frame
                    status = fps_get_one_image(dev_fd,
                                               img_width,
                                               img_height,
                                               DFS747_DUMMY_PIXELS,
                                               raw_buf[f]);
                    if (status < 0) {
                        goto image_mode_test_error;
                    }

                    // Turn off TGEN
                    status = fps_enable_tgen(dev_fd, 0);
                    if (status < 0) {
                        goto image_mode_test_error;
                    }
                }

                // Average each frame
                for (i = 0; i < img_size; i++) {
                    pix_sum = 0.0;
                    sqr_sum = 0.0;
                    for (f = 0; f < avg_frame; f++) {
                        pix = (double) img_buf[f][i];
                        pix_sum +=  pix;
                        sqr_sum += (pix * pix);
                    }
                    pix_avg = pix_sum / avg_frame;

                    avg_img[i] = (uint8_t) pix_avg;
                    sqr_img[i] = (sqr_sum / avg_frame) - (pix_avg * pix_avg);
                }

                // Save (averaged) image
                sprintf(file, "%s/%03d_average.bmp", line, n);
                status = save_bmp(file, avg_img, img_size);
                if (status < 0) {
                    goto image_mode_test_error;
                }

                // Calculate finger image average and deviation
                pix_sum = 0.0;
                sqr_sum = 0.0;
                for (i = 0; i < img_size; i++) {
                    pix_sum += (double) avg_img[i];
                    sqr_sum += sqr_img[i];
                }

                fng_avg = pix_sum / img_size;

                // Extract fingerprint
                for (i = 0; i < img_size; i++) {
                    if (avg_img[i] > bkg_img[i]) {
                        enh_img[i] = fng_img[i] = 0xFF - (avg_img[i] - bkg_img[i]);
                    } else {
                        enh_img[i] = fng_img[i] = 0xFF;
                    }
                }

                // Calculate dynamic range
                fng_dr = fng_avg - bkg_avg;

                // Save finger image
                sprintf(file, "%s/%03d_finger.bmp", line, n);
                status = save_bmp(file, fng_img, img_size);
                if (status < 0) {
                    goto image_mode_test_error;
                }

                if (otsu_mul > 0.0) {
                    // Find Otsu threshold
                    otsu_th = find_otsu_th(fng_img, img_size);
                    DEBUG("%s(): Enhancement Th. = %0d\n", __func__, otsu_th);

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
                    DEBUG("%s(): (Before) pix_min = %0d, pix_max = %0d\n", __func__, pix_min, pix_max);

                    // Use contrast/brightness to enhance the image
                    contrast   = ((double) enh_range) / ((double) pix_max - (double) pix_min);
                    brightness = (((double) pix_max + (double) pix_min) / 2) * contrast - 128.0;
                    DEBUG("%s(): contrast = %0.3f, brightness = %0.3f\n", __func__, contrast, brightness);

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
                    DEBUG("%s(): (After) pix_min = %0d, pix_max = %0d\n", __func__, pix_min, pix_max);
                }

                // Save enhanced image
                sprintf(file, "%s/%03d_enhanced.bmp", line, n);
                status = save_bmp(file, enh_img, img_size);
                if (status < 0) {
                    goto image_mode_test_error;
                }

                // Free allocated buffers
                if (enh_img) free(enh_img);
                if (fng_img) free(fng_img);
                if (sqr_img) free(sqr_img);
                if (avg_img) free(avg_img);
                for (f = 0; f < avg_frame; f++) {
                    if (raw_buf[f]) free(raw_buf[f]);
                }
                if (raw_buf) free(raw_buf);
                if (img_buf) free(img_buf);

                // Display DR
                printf("    Image %0d DR = %0.3f\n", n, fng_dr);
            }

            status = gettimeofday(&stop_time, NULL);
            if (status < 0) {
                goto image_mode_test_error;
            }

            elapsed = get_elapsed_ms(&start_time, &stop_time);
            if (elapsed < 0) {
                goto image_mode_test_error;
            }

            printf("    Done!\n");
            printf("\n");
            printf("    Time Elapsed = %0.3f ms\n", elapsed);
        }

        printf("\n");
        printf("Press ENTER key to continue... ");
        (void) getchar();
    }

    return status;

image_mode_test_error :

    // Free allocated buffers
    if (raw_bkg) free(raw_bkg);
    if (enh_img) free(enh_img);
    if (fng_img) free(fng_img);
    if (sqr_img) free(sqr_img);
    if (avg_img) free(avg_img);
    for (f = 0; f < avg_frame; f++) {
        if (raw_buf[f]) free(raw_buf[f]);
    }
    if (raw_buf) free(raw_buf);
    if (img_buf) free(img_buf);

    printf("\n");
    printf("    Failed!\n");
    printf("\n");

    return status;
}

int
detect_mode_test()
{
    int      status        = 0;
    char     key           = 0;
    char     line[CHAR_CNT];
    char     *arg          = NULL;
    int      user_input    = 0;
    uint8_t  addr[9];
    uint8_t  data[9];
    uint16_t cds_offset    = 0;
    uint8_t  detect_th     = 0;
    uint8_t  row_begin     = 0;
    uint8_t  row_end       = 0;
    uint8_t  col_begin     = 0;
    uint8_t  col_end       = 0;
    uint32_t det_width     = 0;
    uint32_t det_height    = 0;
    uint32_t det_size      = 0;
    uint16_t det_frame     = 4;
    int      mode_old      = 0;
    int32_t  test_limit    = -1;
    uint32_t test_cnt      = 0;
    uint32_t test_interval = 1000;
    double   sleep_us      = 0.0;
    uint8_t  int_ctl_old   = 0;
    int      int_enable    = 1;
    int      exit_detect   = 0;
    int      i;
    int      sleep_mul     = 2;

    while (1) {
        clear_console();

        printf("\n");
        printf("==================\n");
        printf(" Detect Mode Test \n");
        printf("==================\n");
        printf("\n");
        printf("    's' frm - Specify suspend interval.                              \n");
        printf("                frm = Suspend interval, and uint is frames.          \n");
        printf("              This number must be decimal.                           \n");
        printf("    't' tst - Specify test times.                                    \n");
        printf("                tst = Test times in this test.                       \n");
        printf("                      If tst < 0, it means wait infinitely.          \n");
        printf("              This number must be decimal.                           \n");
        printf("    'i' ie  - Enable detect interrupt.                               \n");
        printf("                ie = Enable or disable detect interrupt.             \n");
        printf("                  0: Disable.                                        \n");
        printf("                  1: Enable.                                         \n");
        printf("              This number must be decimal.                           \n");
        printf("    'v' ivl - Interval between successive two tests.                 \n");
        printf("                ivl = Interval in msec.                              \n");
        printf("              This number must be decimal.                           \n");
        printf("    'x' ex  - Exit detect mode test if a finger-on event is detected.\n");
        printf("                ex = Exit detect mode test or not.                   \n");
        printf("                  0: Stay in this test.                              \n");
        printf("                  1: Exit this test.                                 \n");
        printf("    'r' ivl - Interval between successive two re-calibration.        \n");
        printf("                ivl = Interval in seconds. 0 means turning off re-   \n");
        printf("                      calibration.                                   \n");
        printf("              This number must be decimal.                           \n");
        printf("    'g'     - Get current Detect Mode test settings.                 \n");
        printf("    ENTER   - Do Detect Mode test based on current settings.         \n");
        printf("    'q'     - Back to main menu.                                     \n");
        printf("\n");
        printf("Pleae enter: ");

        clear_line(line, sizeof(line));
        fgets(line, sizeof(line), stdin);
        key = line[0];

        if (key == 'q') {
            break;
        }

        printf("\n");
        printf("Result:\n");
        printf("\n");

        if (key == 's') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 1) {
                det_frame = (uint16_t) user_input;
                goto detect_mode_test_show_options;
            } else {
                printf("    ERROR: Suspend interval must be >= 1!\n");
            }
        }

        if (key == 't') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input != 0) {
                test_limit = (uint32_t) user_input;
                goto detect_mode_test_show_options;
            } else {
                printf("    ERROR: Test times must NOT be 0!\n");
            }
        }

        if (key == 'i') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 0) {
                int_enable = user_input;
                goto detect_mode_test_show_options;
            } else {
                printf("    ERROR: This option must be either 0 or 1!\n");
            }
        }

        if (key == 'v') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 0) {
                test_interval = (uint32_t) user_input;
                goto detect_mode_test_show_options;
            } else {
                printf("    ERROR: Test Interval must be >= 0!\n");
            }
        }

        if (key == 'x') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 0) {
                exit_detect = user_input;
                goto detect_mode_test_show_options;
            } else {
                printf("    ERROR: This option must be either 0 or 1!\n");
            }
        }

        if (key == 'r') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if (user_input >= 0) {
                recal_secs = (uint32_t) user_input;
                goto detect_mode_test_show_options;
            } else {
                printf("    ERROR: Test Interval must be >= 0!\n");
            }
        }

        if (key == 'g') {

detect_mode_test_show_options :

            printf("    Suspend Interval                 = %0d frames\n", det_frame);
            if (test_limit < 0) {
                printf("    Test Times                       = INFINITE\n");
            } else {
                printf("    Test Times                       = %0d\n", test_limit);
            }
            printf("    Enable Detect Interrupt          = %s\n",        (int_enable  > 0) ? "TRUE" : "FALSE");
            printf("    Exit Detect Mode after Finger-On = %s\n",        (exit_detect > 0) ? "TRUE" : "FALSE");
            printf("    Test Interval                    = %0d msecs\n", test_interval);
            printf("    Re-calibration Interval          = %0d secs\n",  recal_secs);
        }

        if (key == '\n') {
            printf("    Testing...\n");

            addr[0] = DFS747_REG_DET_CDS_CTL_0;
            addr[1] = DFS747_REG_DET_CDS_CTL_1;
            addr[2] = DFS747_REG_V_DET_SEL;
            addr[3] = DFS747_REG_DET_ROW_BEGIN;
            addr[4] = DFS747_REG_DET_ROW_END;
            addr[5] = DFS747_REG_DET_COL_BEGIN;
            addr[6] = DFS747_REG_DET_COL_END;
            addr[7] = DFS747_REG_SUSP_WAIT_F_CYC_H;
            addr[8] = DFS747_REG_SUSP_WAIT_F_CYC_L;

            status = fps_multiple_read(dev_fd, addr, data, 9);
            if (status < 0) {
                goto detect_mode_test_error;
            }

            // Get detect mode CDS Offset setting
            cds_offset = (((uint16_t) (data[0] & 0x80)) << 1) | ((uint16_t) data[1]);
            DEBUG("%s(): CDS Offset = 0x%03X\n", __func__, cds_offset);

            // Get detect mode Detect Threshold
            detect_th = data[2] & 0x3F;
            DEBUG("%s(): Detect Threshold = 0x%02X\n", __func__, detect_th);

            // Get detect window
            row_begin  = data[3];
            row_end    = data[4];
            col_begin  = data[5];
            col_end    = data[6];
            det_width  = col_end - col_begin + 1;
            det_height = row_end - row_begin + 1;
            det_size   = det_width * det_height;
            DEBUG("%s(): Detect Window = %0d (%0dx%0d)\n", __func__, det_size, det_width, det_height);

            // Get suspend interval
            det_frame = (((uint16_t) data[7]) << 8) | ((uint16_t) data[8]);

            // Assume the slowest oscillator frequency is 250KHz, i.e. 4us
            sleep_us = (double) (det_size * (1 + det_frame) * 8 * 4) * sleep_mul;

            DEBUG("%s(): Suspend Interval = %.3fus\n", __func__, sleep_us);

            // Switch to detect mode
#if 0
            status = fps_switch_mode(dev_fd, DFS747_DETECT_MODE, &mode_old);
            if (status < 0) {
                goto detect_mode_test_error;
            }
#else
            status = fps_switch_mode(dev_fd, DFS747_POWER_DOWN_MODE, &mode_old);
            status = fps_switch_mode(dev_fd, DFS747_DETECT_MODE, NULL);
            if (status < 0) {
                goto detect_mode_test_error;
            }
#endif

            // Skip first scan
            status = fps_scan_detect_event(dev_fd, detect_th, cds_offset, sleep_us, int_enable, 0);
            if (status < 0) {
                goto detect_mode_test_error;
            }

            if (test_limit >= 1) {
                for (test_cnt = 0; test_cnt < test_limit; test_cnt++) {
                    status = fps_scan_detect_event(dev_fd, detect_th, cds_offset, sleep_us, int_enable, 0);

                    // Error
                    if (status < 0) {
                        goto detect_mode_test_error;
                    }

                    // Finger-on detected
                    if (status > 0) {
                        printf("    INFO: Finger-on event is detected!\n");
                        detect_cnt++;

                        // Exit current Detect Mode test loop
                        if (exit_detect > 0) {
                            break;
                        }
                    }

                    // Display progress
                    for (i = 0; i <= (test_cnt % 10); i++) {
                        printf(".");
                    }
                    printf("\n");

                    // Sleep for next test
                    usleep(test_interval * MSEC);
                }

                if (detect_cnt == 0) {
                    printf("    ERROR: No finger-on event!\n");
                }
            } else {
                poll_stop = 0;
                signal(SIGINT, ctrl_c_handler);

                recal_done = 0;
                signal(SIGALRM, recalibrate_detect3);
                alarm(recal_secs);

                detect_cnt = 0;
                while (poll_stop == 0) {
                    status = fps_scan_detect_event(dev_fd, detect_th, cds_offset, -1000, 1, 0);

                    // Error
                    if (status < 0) {
                        if (recal_done != 0) {
                            recal_done = 0;
                            continue;
                        }
                        goto detect_mode_test_error;
                    }

                    // Finger-on detected
                    if (status > 0) {
                        time_t    raw_time;
                        struct tm *time_info;
                        char      time_str[80];

                        time(&raw_time);
                        time_info = localtime(&raw_time);
                        strftime(time_str, sizeof(time_str), "%F %T", time_info);

                        printf("    INFO: Finger-on event is detected! Count = %0d (%s)\n",
                               detect_cnt++, time_str);
                    }

                    // If no finger-on detected...
                }

                poll_stop  = 0;
                recal_done = 0;
                alarm(0);
            }

            printf("    Done!\n");
        }

        printf("\n");
        printf("Press ENTER key to continue... ");
        (void) getchar();
    }

    return status;

detect_mode_test_error :

    printf("\n");
    printf("    Failed!\n");
    printf("\n");

    return status;
}

int
powerdown_mode_test()
{
    int  status   = 0;
    int  mode_old = 0;

    while (1) {
        clear_console();

        printf("\n");
        printf("==================\n");
        printf(" Enter Power-Down \n");
        printf("==================\n");
        printf("\n");
        printf("Result:\n");
        printf("\n");

        printf("    Powering down sensor...\n");

        status = fps_switch_mode(dev_fd, DFS747_POWER_DOWN_MODE, &mode_old);
        if (status < 0) {
            goto powerdown_mode_test_error;
        }

        status = fps_single_write(dev_fd, 0x05, 0x00);
        if (status < 0) {
            goto powerdown_mode_test_error;
        }

        printf("    Done!\n");

        printf("\n");
        printf("Press ENTER key to continue... ");
        (void) getchar();

        break;
    }

    return status;

powerdown_mode_test_error :

    printf("\n");
    printf("    Failed!\n");
    printf("\n");

    return status;
}

int
reset_sensor()
{
    int      status = 0;
    uint32_t delay  = 10 * MSEC;
    int      i;

    while (1) {
        clear_console();

        printf("\n");
        printf("==============\n");
        printf(" Reset Sensor \n");
        printf("==============\n");
        printf("\n");
        printf("Result:\n");
        printf("\n");

        printf("    Resetting sensor...\n");

        for (i = 0; i < 2; i++) {
            status = fps_reset_sensor(dev_fd, 0);
            if (status < 0) {
                return status;
            }
            usleep(delay);

            status = fps_reset_sensor(dev_fd, 1);
            if (status < 0) {
                return status;
            }
            usleep(delay);
        }

        printf("    Done!\n");

        printf("\n");
        printf("Press ENTER key to continue... ");
        (void) getchar();

        break;
    }

    return status;
}

int
set_debug_level()
{
    char key        = 0;
    char line[CHAR_CNT];
    char *arg       = NULL;
    int  user_input = 0;

    while (1) {
        clear_console();

        printf("\n");
        printf("=================\n");
        printf(" Set Debug Level \n");
        printf("=================\n");
        printf("\n");
        printf("    's' lvl - Set debug level.                          \n");
        printf("                lvl = debug level.                      \n");
        printf("                  0: Error.                             \n");
        printf("                  1: Error, Warning.                    \n");
        printf("                  2: Error, Warning, Information.       \n");
        printf("                  3: Error, Warning, Information, Debug.\n");
        printf("                  4: +Register access detail.           \n");
        printf("    'g'     - Get current specify debug level.          \n");
        printf("    'q'     - Back to main menu.                        \n");
        printf("\n");
        printf("Pleae enter: ");

        clear_line(line, sizeof(line));
        fgets(line, sizeof(line), stdin);
        key = line[0];

        if (key == 'q') {
            break;
        }

        printf("\n");
        printf("Result:\n");
        printf("\n");

        if (key == 's') {
            arg        = strtok(line, " ");
            arg        = strtok(NULL, " ");
            user_input = strtol(arg, NULL, 10);

            if ((user_input >= 0) && (user_input <= 4)) {
                debug_level = user_input;
                goto set_debug_level_show_options;
            } else {
                printf("    ERROR: Debug level must be 0 <= level <= 4!\n");
            }
        }

        if (key == 'g') {

set_debug_level_show_options :

            printf("    Debug Level = %0d\n", debug_level);
        }

        printf("\n");
        printf("Press ENTER key to continue... ");
        (void) getchar();
    }

    return 0;
}

int
get_build_date()
{
    clear_console();

    printf("\n");
    printf("========================\n");
    printf(" Get Program Build Date \n");
    printf("========================\n");
    printf("\n");
    printf("Result:\n");
    printf("\n");

    printf("    Built at %s, %s.\n", __TIME__, __DATE__);

    printf("\n");
    printf("Press ENTER key to continue... ");
    (void) getchar();

    return 0;
}

int
quit_program()
{
    printf("\n");
    printf("Bye...\n");
    printf("\n");
    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// Main Program
//

typedef int (*menu_function_t) ();

typedef struct menu_item_s {
    char             key;
    char            *desc;
    menu_function_t  func;
}
menu_item_t;

menu_item_t menu[] = {
    { .key = 'o', .desc = "Open the sensor",             .func = open_sensor         },
    { .key = 'c', .desc = "Close the sensor",            .func = close_sensor        },
    { .key = 'i', .desc = "Initialize the sensor",       .func = init_sensor         },
    { .key = 'r', .desc = "Read register(s)",            .func = read_register       },
    { .key = 'w', .desc = "Write register(s)",           .func = write_register      },
    { .key = 'd', .desc = "Dump all registers",          .func = dump_register       },
    { .key = 'k', .desc = "Calibrate image (method 1)",  .func = calibrate_image1    },
    { .key = '2', .desc = "Calibrate image (method 2)",  .func = calibrate_image2    },
    { .key = 'K', .desc = "Calibrate detect (method 1)", .func = calibrate_detect1   },
    { .key = '3', .desc = "Calibrate detect (method 2)", .func = calibrate_detect2   },
    { .key = '4', .desc = "Calibrate detect (method 3)", .func = calibrate_detect3   },
    { .key = '5', .desc = "Calibrate detect (method 4)", .func = calibrate_detect4   },
    { .key = 'I', .desc = "Image Mode test",             .func = image_mode_test     },
    { .key = 'D', .desc = "Detect Mode test",            .func = detect_mode_test    },
    { .key = 'p', .desc = "Power-Down Mode test",        .func = powerdown_mode_test },
    { .key = 'R', .desc = "Reset the sensor",            .func = reset_sensor        },
    { .key = 'l', .desc = "Set debug level",             .func = set_debug_level     },
    { .key = 'b', .desc = "Get program build date",      .func = get_build_date      },
    { .key = 'q', .desc = "Quit this program",           .func = quit_program        },
    // TODO: adding more functions here...
    { .key = 0x00, .desc = NULL, .func = NULL }
};

int
main (int argc, char *argv[])
{
    char key = 0;
    int  i;

    tzset();

    while (key != 'q') {
        key = 0;

        clear_console();

        printf("\n");
        printf("===========\n");
        printf(" Main Menu \n");
        printf("===========\n");
        printf("\n");

        for (i = 0; menu[i].key != 0x00; i++) {
            printf("    '%c' - %s\n", menu[i].key, menu[i].desc);
        }

        printf("\n");
        printf("Pleae select: ");

        key = get_key();
        if (key == '\n') {
            continue;
        }

        for (i = 0; menu[i].key != 0x00; i++) {
            if (key == menu[i].key) {
                if ((dev_fd == 0) && // Device is not opened
                    (key != 'o')  && // key = Open sensor
                    (key != 'q')  && // key = Quick program
                    (key != 'l')  && // key = Set debug level
                    (key != 'b')) {  // key = Get program build date
                    printf("\n");
                    printf("    ERROR: Sensor should be opened first!\n");
                    printf("\n");
                    sleep(1);
                    break;
                }

                if (menu[i].func() < 0) {
                    exit(-1);
                }
                break;
            }
        }
    }

    return 0;
}
