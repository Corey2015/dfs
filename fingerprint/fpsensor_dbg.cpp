#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/types.h>
#include "fps_driver.h"
#include <sys/stat.h>


////////////////////////////////////////////////////////////////////////////////
//
// Detect Calibration
//

#define DFS747_CDS_SEARCH_START (0x01FF)
#define DFS747_CDS_SEARCH_STEP  (1)
#define DFS747_CDS_SEARCH_COUNT ((DFS747_CDS_SEARCH_START + 1) / DFS747_CDS_SEARCH_STEP)


////////////////////////////////////////////////////////////////////////////////
//
// Global Variables
//

static int     debug_level = 3;
static char    dev_path[]  = "/dev/dfs0";
static int     dev_fd      = 0;
static uint8_t bkg_img[DFS747_SENSOR_SIZE];


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

    struct dfs747_ioc_transfer tr;
    tr.len    = state;
    tr.opcode = DFS747_IOC_RESET_SENSOR;

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
    struct dfs747_ioc_transfer tr;

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

     tr.tx_buf = (unsigned long) tx;
     tr.rx_buf = (unsigned long) rx;
     tr.len    = len;
     tr.opcode = DFS747_IOC_REGISTER_MASS_READ;

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

    struct dfs747_ioc_transfer tr;
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

     tr.tx_buf = (unsigned long) tx;
     tr.len    = (len * 2);
     tr.opcode = DFS747_IOC_REGISTER_MASS_WRITE,

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
    int      i;
    uint32_t img_size;
    uint8_t  tx[6];
    struct dfs747_ioc_transfer tr;

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
    // NOTE: Checking frame-ready event is implemented at driver level.
        tr.tx_buf = (unsigned long) tx,
        tr.rx_buf = (unsigned long) img;
        tr.len    = img_size;
        tr.opcode = DFS747_IOC_GET_ONE_IMG;

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
    int     i;
    uint8_t addr[3] = {0x02, 0x03, 0x04};
    uint8_t data[3] = {0x00, 0x00, 0x00};

    status = fps_multiple_read(fd, addr, data, sizeof(addr));
    if (status < 0) {
        return status;
    }

    if (data[0] & DFS747_ENABLE_DETECT) {
        *mode_old = DFS747_DETECT_MODE;
    } else if (data[1] == 0xFF) {
        *mode_old = DFS747_POWER_DOWN_MODE;
    } else {
        *mode_old = DFS747_IMAGE_MODE;
    }

    switch (mode_new) {
        case DFS747_IMAGE_MODE : {
            data[0] &= ~DFS747_ENABLE_DETECT;
            data[1]  = 0x84;
            data[2] &= ~(1 << 0);
            break;
        }

        case DFS747_DETECT_MODE : {
            data[0] |= DFS747_ENABLE_DETECT;
            break;
        }

        case DFS747_POWER_DOWN_MODE : {
            data[0] &= ~DFS747_ENABLE_DETECT;
            data[1]  = 0xFF;
            data[2] |= (1 << 0);
            break;
        }

        default : return -1;
    }

    status = fps_multiple_write(fd, addr, data, sizeof(addr));
    if (status < 0) {
        return status;
    }

    return status;
}

static int
fps_enable_tgen(const int fd,
                const int enable)
{
    int     status = 0;
    int     i;
    uint8_t addr;
    uint8_t data;

    addr = DFS747_REG_GBL_CTL;
    data = 0x00;

    status = fps_single_read(fd, addr, &data);
    if (status < 0) {
        return status;
    }

    if (enable != 0) {
        data |=  DFS747_ENABLE_TGEN;
    } else {
        data &= ~DFS747_ENABLE_TGEN;
    }

    status = fps_single_write(fd, addr, data);
    if (status < 0) {
        return status;
    }

    return status;
}

static int
fps_scan_detect_event(const int      fd,
                      const uint8_t  detect_th,
                      const uint16_t cds_offset,
                      const double   sleep_us,
                      const int      int_enable,
                      const int      cal_detect)
{
    int           status = 0;
    uint8_t       event  = 0;
    uint8_t       addr[3];
    uint8_t       data[3];
    struct pollfd poll_fps;

    // Disable interrupt and clear pending events
    addr[0] = DFS747_REG_INT_CTL;
    addr[1] = DFS747_REG_INT_EVENT;
    data[0] = 0x00;
    data[1] = 0x00;

    status = fps_multiple_write(dev_fd, addr, data, 2);
    if (status < 0) {
        return -1;
    }

    if (cal_detect > 0) {
        // Set up Detect Threshold and CDS offset
        addr[0] = DFS747_REG_V_DET_SEL;
        addr[1] = DFS747_REG_DET_CDS_CTL_0;
        addr[2] = DFS747_REG_DET_CDS_CTL_1;
       
        status = fps_multiple_read(dev_fd, addr, data, 3);
        if (status < 0) {
            return -1;
        }
       
        data[0] = detect_th & 0x3F;
        data[1] = ((uint8_t) ((cds_offset & 0x0100) >> 1)) | (data[1] & 0x7F);
        data[2] =  (uint8_t)  (cds_offset & 0x00FF);
       
        status = fps_multiple_write(dev_fd, addr, data, 4);
        if (status < 0) {
            return -1;
        }

        usleep(sleep_us);
    }

    if (int_enable > 0) {
        // Turn on Detect interrupt
        status = fps_single_write(dev_fd, DFS747_REG_INT_CTL, DFS747_DETECT_EVENT);
        if (status < 0) {
            return -1;
        }

#if defined(USE_POLL_METHOD)
        poll_fps.fd      = dev_fd;
        poll_fps.events  = POLLIN /*| POLLRDNORM*/;
        poll_fps.revents = 0;
        status = poll(&poll_fps, 1, (int) (sleep_us / 1000));
        if (status < 0) {
            ERROR("%s(): Calling poll() failed! status = %0d\n", __func__, status);
            return -1;
        }
 //       DEBUG("%s(): poll_fps.revents = %0d\n", __func__, poll_fps.revents);

        event = (poll_fps.revents != 0) ? DFS747_DETECT_EVENT : 0;
#else
        // Sleep to wait for event or interrupts
        usleep((uint32_t) sleep_us);

        status = fps_single_read(dev_fd, DFS747_REG_INT_EVENT, &event);
        if (status < 0) {
            return -1;
        }
#endif

        // Turn off Detect interrupt
        status = fps_single_write(dev_fd, DFS747_REG_INT_CTL, 0x00);
        if (status < 0) {
            return -1;
        }
    } else {
        // Sleep to wait for event or interrupts
        usleep((uint32_t) sleep_us);

        status = fps_single_read(dev_fd, DFS747_REG_INT_EVENT, &event);
        if (status < 0) {
            return -1;
        }
    }

    status = fps_single_write(dev_fd, DFS747_REG_INT_EVENT, 0x00);
    if (status < 0) {
        return -1;
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

        for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
            status = fps_scan_detect_event(fd, detect_th, cds_middle, sleep_us, int_enable, 1);

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
        ERROR("%s(): Maximum CDS Offset reached!\n", __func__);
        return -1;
    }

    if (*cds_offset == lower) {
        ERROR("%s(): Minimum CDS Offset reached!\n", __func__);
        return -1;
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

        for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
            status = fps_scan_detect_event(fd, det_middle, cds_offset, sleep_us, int_enable, 1);

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
        ERROR("%s(): Maximum Detect Threshold reached!\n", __func__);
        return -1;
    }

    if (*detect_th == lower) {
        ERROR("%s(): Minimum Detect Threshold reached!\n", __func__);
        return -1;
    }

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
//
// Menu Functions
//

int
open_sensor()
{
    int  status = 0;
    char key    = 0;

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
            return status;
        }

        printf("    Done!\n");
        printf("\n");
        printf("Press 'q' to back to main menu, or ENTER to do it again: ");

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
close_sensor()
{
    int  status = 0;
    char key    = 0;

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
            return status;
        }

        printf("    Done!\n");
        printf("\n");
        printf("Press 'q' to back to main menu, or ENTER to do it again: ");

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
init_sensor()
{
    int     status = 0;
    char    key    = 0;
    uint8_t addr[] = {0x01, 0x00, 0x02, 0x04, 0x07, 0x08, 0x1D};
    uint8_t data[] = {0x00, 0x00, 0x00, 0x20, 0xFC, 0x03, 0xFF};
    int     i;

    for (i = 0; i < sizeof(bkg_img); i++) {
        bkg_img[i] = 0x00;
    }

    while (1) {
        clear_console();

        printf("\n");
        printf("===================\n");
        printf(" Initialize Sensor \n");
        printf("===================\n");
        printf("\n");
        printf("Result:\n");
        printf("\n");
        printf("    Initializing sensor...\n");

        fps_single_write(dev_fd, 0x05, 0x00);
        usleep(1 * MSEC);
        fps_single_write(dev_fd, 0x05, 0x08);
        usleep(1 * MSEC);
        fps_single_write(dev_fd, 0x05, 0x04);
        usleep(1 * MSEC);
        fps_single_write(dev_fd, 0x05, 0x0F);
        usleep(1 * MSEC);

        status = fps_multiple_write(dev_fd, addr, data, sizeof(addr));
        if (status < 0) {
            return status;
        }

        printf("    Done!\n");
        printf("\n");
        printf("Press 'q' to back to main menu, or ENTER to do it again: ");

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
calibrate_image()
{
    int            status         = 0;
    char           key            = 0;
    char           line[CHAR_CNT];
    char           *arg           = NULL;
    uint8_t        upper_bond     = 240;
    uint8_t        lower_bond     = 10;
    uint8_t        first_row      = 2;
    uint8_t        middle_row     = DFS747_SENSOR_ROWS / 2;
    uint8_t        last_row       = DFS747_SENSOR_ROWS - 2;
    uint8_t        col_scan_begin = 4;
    uint8_t        col_scan_end   = DFS747_SENSOR_COLS - 4;
    uint8_t        addr[4];
    uint8_t        data[4];
    int            mode_old       = 0;
    uint8_t        row_begin      = 0;
    uint8_t        row_end        = DFS747_SENSOR_ROWS - 1;
    uint8_t        col_begin      = 0;
    uint8_t        col_end        = DFS747_SENSOR_COLS - 1;
    uint32_t       img_width      = DFS747_SENSOR_COLS;
    uint32_t       img_height     = DFS747_SENSOR_ROWS;
    uint32_t       img_size       = (DFS747_SENSOR_COLS * DFS747_SENSOR_ROWS);
    uint8_t        *raw_buf       = NULL;
    uint8_t        *img_buf       = NULL;
    uint16_t       cds_offset     = DFS747_MAX_CDS_OFFSET;
    uint8_t        pga_gain       = DFS747_MAX_PGA_GAIN;
    int            pix_gt_lower   = 1;
    int            pix_lt_upper   = 1;
    struct timeval start_time;
    struct timeval stop_time;
    double         elapsed        = 0.0;
    int            cal_success    = 0;
    int            i;

    while (1) {
        clear_console();

        printf("\n");
        printf("=================\n");
        printf(" Calibrate Image \n");
        printf("=================\n");
        printf("\n");
        printf("    'b' ub lb    - Set target upper and lower bond of calibrated pixels.\n");
        printf("                     frm = number of frames to average an image.        \n");
        printf("                     num = Number of image to acquire.                  \n");
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

        if (key == 'b') {
            arg = strtok(line, " ");

            arg        = strtok(NULL, " ");
            upper_bond = strtol(arg, NULL, 10);
            arg        = strtok(NULL, " ");
            lower_bond = strtol(arg, NULL, 10);

            printf("\n");
            printf("Result:\n");
            printf("\n");

            if (upper_bond >= lower_bond) {
                printf("    Calibration Bond (Upper/Lower) = %0d/%0d\n", upper_bond, lower_bond);
            } else {
                printf("    ERROR: Upper bond must be greater than or equal to lower bond!\n");
                upper_bond = 240;
                lower_bond = 10;
            }
            printf("\n");
        }

        if (key == 'r') {
            arg = strtok(line, " ");

            arg        = strtok(NULL, " ");
            first_row  = strtol(arg, NULL, 10);
            arg        = strtok(NULL, " ");
            middle_row = strtol(arg, NULL, 10);
            arg        = strtok(NULL, " ");
            last_row   = strtol(arg, NULL, 10);

            printf("\n");
            printf("Result:\n");
            printf("\n");

            if ((first_row < middle_row) && (middle_row < last_row)) {
                printf("    Rows (First/Middle/Last)       = %0d/%0d/%0d\n", first_row, middle_row, last_row);
            } else {
                printf("    ERROR: Must satisfy First < Middle < Last!\n");
                first_row  = 2;
                middle_row = DFS747_SENSOR_ROWS / 2;
                last_row   = DFS747_SENSOR_ROWS - 2;
            }
            printf("\n");
        }

        if (key == 'c') {
            arg = strtok(line, " ");

            arg            = strtok(NULL, " ");
            col_scan_begin = strtol(arg, NULL, 10);
            arg            = strtok(NULL, " ");
            col_scan_end   = strtol(arg, NULL, 10);

            printf("\n");
            printf("Result:\n");
            printf("\n");

            if (col_scan_end >= col_scan_begin) {
                printf("    Column Scan (Begin/End)        = %0d/%0d\n", col_scan_begin, col_scan_end);
            } else {
                printf("    ERROR: Scan end point must be greater than or equal to begin point!\n");
                col_scan_begin = 10;
                col_scan_end   = DFS747_SENSOR_COLS - 10;
            }
            printf("\n");
        }

        if (key == 'g') {
            printf("\n");
            printf("Result:\n");
            printf("\n");
            printf("    Calibration Bond (Upper/Lower) = %0d/%0d\n",     upper_bond, lower_bond);
            printf("    Rows (First/Middle/Last)       = %0d/%0d/%0d\n", first_row, middle_row, last_row);
            printf("    Column Scan (Begin/End)        = %0d/%0d\n",     col_scan_begin, col_scan_end);
            printf("\n");
        }

        if (key == '\n') {
            printf("\n");
            printf("Result:\n");
            printf("\n");
            printf("    Calibrating...\n");
            printf("\n");

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
                goto calibrate_image_error;
            }

            img_width  = col_end - col_begin + 1;
            img_height = row_end - row_begin + 1;
            img_size   = img_width * img_height;

            raw_buf = (uint8_t *) malloc(img_size + DFS747_DUMMY_PIXELS);
            if (raw_buf == NULL) {
                status = -1;
                goto calibrate_image_error;
            }
            img_buf = &raw_buf[DFS747_DUMMY_PIXELS];

            addr[0] = DFS747_REG_IMG_CDS_CTL_0;
            addr[1] = DFS747_REG_IMG_CDS_CTL_1;
            addr[2] = DFS747_REG_IMG_PGA1_CTL;

            cds_offset = DFS747_MAX_CDS_OFFSET;
            pga_gain   = DFS747_MAX_PGA_GAIN;

            // Switch to image mode
            status = fps_switch_mode(dev_fd, DFS747_IMAGE_MODE, &mode_old);
            if (status < 0) {
                goto calibrate_image_error;
            }

            status = gettimeofday(&start_time, NULL);
            if (status < 0) {
                goto calibrate_image_error;
            }

            while (1) {
                DEBUG("CDS Offset = 0x%03X, PGA Gain = 0x%02X\n", cds_offset, pga_gain);

                // Set CDS Offset and PGA Gain
                status = fps_multiple_read(dev_fd, addr, data, 3);
                if (status < 0) {
                    goto calibrate_image_error;
                }

                data[0] = ((uint8_t) ((cds_offset & 0x0100) >> 1)) | (data[0] & 0x7F);
                data[1] =  (uint8_t)  (cds_offset & 0x00FF);
                data[2] = pga_gain & 0x0F;

                status = fps_multiple_write(dev_fd, addr, data, 3);
                if (status < 0) {
                    goto calibrate_image_error;
                }

                // Clear all pending events
                status = fps_single_write(dev_fd, DFS747_REG_INT_EVENT, 0x00);
                if (status < 0) {
                    goto calibrate_image_error;
                }

                // Turn on TGEN
                status = fps_enable_tgen(dev_fd, 1);
                if (status < 0) {
                    goto calibrate_image_error;
                }

                // Get a frame
                status = fps_get_one_image(dev_fd,
                                           img_width,
                                           img_height,
                                           DFS747_DUMMY_PIXELS,
                                           raw_buf);
                if (status < 0) {
                    goto calibrate_image_error;
                }

                // Turn off TGEN
                status = fps_enable_tgen(dev_fd, 0);
                if (status < 0) {
                    goto calibrate_image_error;
                }

                pix_gt_lower = 1;
                pix_lt_upper = 1;

                for (i = col_scan_begin; i <= col_scan_end; i++) {
                    if (img_buf[first_row  * DFS747_SENSOR_ROWS + i] > upper_bond) {
                        pix_lt_upper = 0;
                    }

                    if (img_buf[middle_row * DFS747_SENSOR_ROWS + i] < lower_bond) {
                        pix_gt_lower = 0;
                    }

                    if (img_buf[last_row   * DFS747_SENSOR_ROWS + i] > upper_bond) {
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
                goto calibrate_image_error;
            }

            elapsed = get_elapsed_ms(&start_time, &stop_time);
            if (elapsed < 0) {
                goto calibrate_image_error;
            }

            // Switch back to original mode
            status = fps_switch_mode(dev_fd, mode_old, &mode_old);
            if (status < 0) {
                goto calibrate_image_error;
            }

            if (cal_success > 0) {
                memcpy(bkg_img, img_buf, img_size);

                printf("    Done!\n");
                printf("\n");
                printf("    CDS Offset = 0x%03X\n", cds_offset);
                printf("    PGA Gain   = 0x%02X\n", pga_gain);
                printf("\n");
                printf("    Time elapsed = %0.3f ms\n", elapsed);
                printf("\n");
            } else {
                printf("    Failed!\n");
                printf("\n");
            }

            // Free allocated buffers
            if (raw_buf) free(raw_buf);
        }

        printf("Press ENTER key to continue... ");
        (void) getchar();
    }

    return status;

calibrate_image_error :

    // Free allocated buffers
    if (raw_buf) free(raw_buf);

    printf("    Failed!\n");
    printf("\n");

    return status;
}

int
calibrate_detect()
{
    int            status           = 0;
    char           key              = 0;
    char           line[CHAR_CNT];
    char           *arg             = NULL;
    uint16_t       det_frame        = 1;
    uint32_t       scan_limit       = 8;
    uint32_t       scan_cnt         = 0;
    uint8_t        addr[4];
    uint8_t        data[4];
    int            mode_old         = 0;
    uint8_t        row_begin        = DFS747_SENSOR_ROWS / 2;
    uint8_t        row_end          = DFS747_SENSOR_ROWS / 2;
    uint8_t        col_begin        = (DFS747_SENSOR_COLS / 2) - 8;
    uint8_t        col_end          = (DFS747_SENSOR_COLS / 2) + 7;
    uint32_t       det_width        = 16;
    uint32_t       det_height       = 1;
    uint32_t       det_window       = 16;
    uint16_t       cds_offset       = DFS747_MAX_CDS_OFFSET;
    uint8_t        detect_th        = DFS747_MAX_DETECT_TH;
    double         sleep_us         = 0.0;
    struct timeval start_time;
    struct timeval stop_time;
    double         elapsed          = 0.0;
    uint16_t       extra_cds_offset = 4;
    int            int_enable       = 1;
    int            fine_tune_enable = 0;
    int            cal_success      = 0;
    uint8_t        detect_th_array[DFS747_CDS_SEARCH_COUNT];
    uint32_t       curr_cds_range   = 0;
    uint32_t       max_cds_range    = 0;
    int            i;

    while (1) {
        clear_console();

        printf("\n");
        printf("==================\n");
        printf(" Calibrate Detect \n");
        printf("==================\n");
        printf("\n");
        printf("    's' frm scn ac ie fn - Set detect caibration parameters.               \n");
        printf("                             frm = Suspend interval in frames.             \n");
        printf("                             scn = How many detect scans in one iteration. \n");
        printf("                             ac  = Additional CDS Offset to be added.      \n");
        printf("                             ie  = Enable detect interrupt.                \n");
        printf("                             fn  = Enable fine-tune stage.                 \n");
        printf("                           These numbers must be decimal.                  \n");
        printf("    'g'                  - Get current detect calibration settings.        \n");
        printf("    ENTER                - Do detect calibration based on current settings.\n");
        printf("    'q'                  - Back to main menu.                              \n");
        printf("\n");
        printf("Pleae enter: ");

        clear_line(line, sizeof(line));
        fgets(line, sizeof(line), stdin);
        key = line[0];

        if (key == 'q') {
            break;
        }

        if (key == 's') {
            arg = strtok(line, " ");

            arg              = strtok(NULL, " ");
            det_frame        = strtol(arg, NULL, 10);
            arg              = strtok(NULL, " ");
            scan_limit       = strtol(arg, NULL, 10);
            arg              = strtok(NULL, " ");
            extra_cds_offset = strtol(arg, NULL, 10);
            arg              = strtok(NULL, " ");
            int_enable       = strtol(arg, NULL, 10);
            arg              = strtok(NULL, " ");
            fine_tune_enable = strtol(arg, NULL, 10);

            printf("\n");
            printf("Result:\n");
            printf("\n");

            if (det_frame >= 1) {
                printf("    Suspend Interval        = %0d frames\n", det_frame);
            } else {
                printf("    ERROR: Interval must be greater than or equal to 1!\n");
                det_frame = 1;
            }

            if (scan_limit >= 1) {
                printf("    Scan Limit              = %0d\n", scan_limit);
            } else {
                printf("    ERROR: Scan limit mut be greater than or equal to 1!\n");
                scan_limit = 1;
            }

            printf("    Additional CDS Offset   = %0d\n", extra_cds_offset);
            printf("    Enable Detect Interrupt = %s\n",  (int_enable       > 0) ? "TRUE" : "FALSE");
            printf("    Enable Fine Tune Stage  = %s\n",  (fine_tune_enable > 0) ? "TRUE" : "FALSE");
            printf("\n");
        }

        if (key == 'g') {
            printf("\n");
            printf("Result:\n");
            printf("\n");
            printf("    Suspend Interval        = %0d frames\n", det_frame);
            printf("    Scan Limit              = %0d\n",        scan_limit);
            printf("    Additional CDS Offset   = %0d\n",        extra_cds_offset);
            printf("    Enable Detect Interrupt = %s\n",         (int_enable       > 0) ? "TRUE" : "FALSE");
            printf("    Enable Fine Tune Stage  = %s\n",         (fine_tune_enable > 0) ? "TRUE" : "FALSE");
            printf("\n");
        }

        if (key == '\n') {
            printf("\n");
            printf("Result:\n");
            printf("\n");
            printf("    Calibrating...\n");
            printf("\n");

            // Disable all interrupts
            status = fps_single_write(dev_fd, DFS747_REG_INT_CTL, 0x00);
            if (status < 0) {
                goto calibrate_detect_error;
            }

            // Set image window
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
                goto calibrate_detect_error;
            }

            // Set suspend interval
            addr[0] = DFS747_REG_SUSP_WAIT_F_CYC_H;
            addr[1] = DFS747_REG_SUSP_WAIT_F_CYC_L;

            data[0] = (uint8_t) ((det_frame & 0xFF00) >> 8);
            data[1] = (uint8_t) ((det_frame & 0x00FF) >> 0);

            status = fps_multiple_write(dev_fd, addr, data, 2);
            if (status < 0) {
                goto calibrate_detect_error;
            }

            det_width  = col_end - col_begin + 1;
            det_height = row_end - row_begin + 1;
            det_window = det_width * det_height;

            sleep_us = (double) (det_window * (1 + det_frame) * 10 * 8 * 2500) / 1000;
            if (sleep_us < 20.0 * MSEC) {
                sleep_us = 20.0 * MSEC;
            }

            // Switch to detect mode
            status = fps_switch_mode(dev_fd, DFS747_DETECT_MODE, &mode_old);
            if (status < 0) {
                goto calibrate_detect_error;
            }
           
            status = gettimeofday(&start_time, NULL);
            if (status < 0) {
                goto calibrate_detect_error;
            }

            // Skip first scan
            status = fps_scan_detect_event(dev_fd, detect_th, cds_offset, sleep_us, int_enable, 1);
            if (status < 0) {
                goto calibrate_detect_error;
            }

            status = fps_search_detect_threshold(dev_fd, cds_offset,
                                                 DFS747_MAX_DETECT_TH, DFS747_MIN_DETECT_TH,
                                                 sleep_us, scan_limit, int_enable,
                                                 &detect_th_array[0]);
            if (status < 0) {
                goto calibrate_detect_error;
            }
            DEBUG("%s(): CDS Offset = 0x%03X, Detect Th. = 0x%02X\n", __func__, cds_offset, detect_th_array[0]);

            cal_success = 1;
            for (i = 1; i < DFS747_CDS_SEARCH_COUNT; i++) {
                cds_offset = DFS747_CDS_SEARCH_START - (i * DFS747_CDS_SEARCH_STEP);
                detect_th  = detect_th_array[i - 1];

                while (1) {
                    status = fps_scan_detect_event(dev_fd, detect_th, cds_offset, sleep_us, int_enable, 1);
                    
                    // Finger-on is still detected
                    if (status > 0) {
                        detect_th++;
                        DEBUG("%s(): detect_th++!\n", __func__);
                        if (detect_th == DFS747_MAX_DETECT_TH) {
                            cal_success = 0;
                            break;
                        }
                    }

                    // As expected, no finger-on detected
                    if (status == 0) {
                        break;
                    }

                    // Error
                    if (status < 0) {
                        goto calibrate_detect_error;
                    }
                }

                detect_th_array[i] = detect_th;
                DEBUG("%s(): CDS Offset = 0x%03X, Detect Th. = 0x%02X\n", __func__, cds_offset, detect_th);
            }

            curr_cds_range = 0;
            max_cds_range  = 0;

            for (i = 0; i < (DFS747_CDS_SEARCH_COUNT - 1); i++) {
                curr_cds_range++;
                if (detect_th_array[i] != detect_th_array[i + 1]) {
                    if (curr_cds_range > max_cds_range) {
                        max_cds_range = curr_cds_range;
                        cds_offset = DFS747_CDS_SEARCH_START - (i * DFS747_CDS_SEARCH_STEP);
                        DEBUG("%s(): Detect Th = 0x%02X, CDS Offset = 0x%03X, Count = %0d\n",
                              __func__, detect_th_array[i], cds_offset, max_cds_range);
                    }

                    curr_cds_range = 0;
                }
            }

            // Lookup Detect Threshold from CDS Offset
            detect_th = detect_th_array[(DFS747_CDS_SEARCH_START - cds_offset) / DFS747_CDS_SEARCH_STEP];
            
            if (fine_tune_enable > 0) {
                while (1) {
                    scan_cnt = 0;
                    for (i = 0; i < scan_limit; i++) {
                        status = fps_scan_detect_event(dev_fd, detect_th, cds_offset, sleep_us, int_enable, 1);
                        
                        // Finger-on is still detected
                        if (status > 0) {
                            scan_cnt++;
                        }
                   
                        // Error
                        if (status < 0) {
                            goto calibrate_detect_error;
                        }
                    }
               
#if 0
                    // No events detected in scan_limit times
                    if (scan_cnt == 0) {
                        if (cds_offset == DFS747_MIN_CDS_OFFSET) {
                            cal_success = 0;
                            break;
                        }

                        cds_offset--;
                        DEBUG("%s(): cds_offset--!\n", __func__);
                    } else {
                        break;
                    }
#endif

                    // No events detected in scal_limit times
                    if (scan_cnt == 0) {
                        if (cds_offset != DFS747_MIN_CDS_OFFSET) {
                            DEBUG("%s(): cds_offset--!\n", __func__);
                            cds_offset--;
                        } else if (detect_th != DFS747_MIN_DETECT_TH) {
                            DEBUG("%s(): detect_th--!\n", __func__);
                            detect_th--;
                        } else {
                            DEBUG("%s(): No more settings!\n", __func__);
                            cal_success = 0;
                            break;
                        }
                        continue;
                    }

                    // Events always happen in scal_limit times
                    if (scan_cnt == scan_limit) {
                        if (cds_offset != DFS747_MAX_CDS_OFFSET) {
                            DEBUG("%s(): cds_offset++!\n", __func__);
                            cds_offset++;
                        } else if (detect_th != DFS747_MAX_DETECT_TH) {
                            DEBUG("%s(): detect_th++!\n", __func__);
                            detect_th++;
                        } else {
                            DEBUG("%s(): No more settings!\n", __func__);
                            cal_success = 0;
                            break;
                        }
                        continue;
                    }

                    break;
                }
            }

            // Add additional CDS Offset to avoid false trigger
            cds_offset += extra_cds_offset;
            if (cds_offset > DFS747_MAX_CDS_OFFSET) {
                cds_offset  = DFS747_MAX_CDS_OFFSET;
                cal_success = 0;
            }

#if 0
            // Lookup Detect Threshold from CDS Offset
            detect_th = detect_th_array[(DFS747_CDS_SEARCH_START - cds_offset) / DFS747_CDS_SEARCH_STEP];
#endif

            // Fill the result to the registers
            addr[0] = DFS747_REG_DET_CDS_CTL_0;
            addr[1] = DFS747_REG_DET_CDS_CTL_1;
            addr[2] = DFS747_REG_V_DET_SEL;

            status = fps_multiple_read(dev_fd, addr, data, 3);
            if (status < 0) {
                goto calibrate_detect_error;
            }

            data[0] = (data[0] & 0x7F) | ((uint8_t) ((cds_offset & 0x0100) >> 1));
            data[1] = (uint8_t) (cds_offset & 0x00FF);
            data[2] = detect_th;

            status = fps_multiple_write(dev_fd, addr, data, 3);
            if (status < 0) {
                goto calibrate_detect_error;
            }

            status = gettimeofday(&stop_time, NULL);
            if (status < 0) {
                goto calibrate_detect_error;
            }

            elapsed = get_elapsed_ms(&start_time, &stop_time);
            if (elapsed < 0) {
                goto calibrate_detect_error;
            }

            // Switch back to original mode
            status = fps_switch_mode(dev_fd, mode_old, &mode_old);
            if (status < 0) {
                goto calibrate_detect_error;
            }

            if (cal_success > 0) {
                printf("    Done!\n");
                printf("\n");
                printf("    Detect Threshold = 0x%02X\n", detect_th);
                printf("    CDS Offset       = 0x%03X\n", cds_offset);
                printf("\n");
                printf("    Time elapsed = %0.3f ms\n", elapsed);
                printf("\n");
            } else {
                printf("    Failed!\n");
                printf("\n");
            }
        }

        printf("Press ENTER key to continue... ");
        (void) getchar();
    }

    return status;

calibrate_detect_error :

    printf("    Failed!\n");
    printf("\n");

    return status;
}

int
image_mode_test()
{
    int            status     = 0;
    char           key        = 0;
    char           line[CHAR_CNT];
    char           *arg       = NULL;
    uint8_t        avg_frame  = 4;
    uint8_t        num_frame  = 20;
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
    uint8_t        *enh_img   = NULL;
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
    int            n;
    int            i;
    int            j;

    while (1) {
        clear_console();

        printf("\n");
        printf("===========\n");
        printf(" Get Image \n");
        printf("===========\n");
        printf("\n");
        printf("    's' frm num p1 p2 - Set getting image parameters.                  \n");
        printf("                          frm = number of frames to average an image.  \n");
        printf("                          num = Number of image to acquire.            \n");
        printf("                          p1  = Parameter for image enhancement.       \n");
        printf("                          p2  = Parameter for image enhancement.       \n");
        printf("                        p1 must be a floating point number. Others must\n");
        printf("                        be decimal. p2 range: 0 <= p2 <= 255.          \n");
        printf("    'g'               - Get current getting image settings.            \n");
        printf("    'b'               - Get an image and save it as a background.      \n");
        printf("    ENTER             - Get image based on current settings.           \n");
        printf("    'q'               - Back to main menu.                             \n");
        printf("\n");
        printf("Pleae enter: ");

        clear_line(line, sizeof(line));
        fgets(line, sizeof(line), stdin);
        key = line[0];

        if (key == 'q') {
            break;
        }

        if (key == 's') {
            arg = strtok(line, " ");

            arg       = strtok(NULL, " ");
            avg_frame = strtol(arg, NULL, 10);
            arg       = strtok(NULL, " ");
            num_frame = strtol(arg, NULL, 10);
            arg       = strtok(NULL, " ");
            otsu_mul  = strtod(arg, NULL);
            arg       = strtok(NULL, " ");
            enh_range = strtol(arg, NULL, 10);

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

            printf("\n");
            printf("Result:\n");
            printf("\n");
            printf("    Row Begin    = %0d\n", row_begin);
            printf("    Row End      = %0d\n", row_end);
            printf("    Column Begin = %0d\n", col_begin);
            printf("    Column End   = %0d\n", col_end);
            printf("\n");
            printf("    Image Size = %0d (%0dx%0d)\n", img_size, img_width, img_height);
            printf("\n");

            if (avg_frame >= 1) {
                printf("    Number of Frames to Average = %0d\n", avg_frame);
            } else {
                printf("    ERROR: 'Number of Frames to Average' must be >= 1!\n");
                avg_frame = 1;
            }

            if (num_frame >= 1) {
                printf("    Number of Frames to Acquire = %0d\n", num_frame);
            } else {
                printf("    ERROR: 'Number of Frames to Aquire' must be >= 1!\n");
                num_frame = 1;
            }

            printf("\n");
            printf("    Image Enhancement Parameter 1 = %0.3f\n", otsu_mul);
            printf("    Image Enhancement Parameter 2 = %0d\n",   enh_range);
            printf("\n");
        }

        if (key == 'g') {
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

            printf("\n");
            printf("Result:\n");
            printf("\n");
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
            printf("    Image Enhancement Parameter 1 = %0.3f\n", otsu_mul);
            printf("    Image Enhancement Parameter 2 = %0d\n",   enh_range);
            printf("\n");
        }

        if (key == 'b') {
            printf("\n");
            printf("Result:\n");
            printf("\n");
            printf("    Getting Background Image...\n");
            printf("\n");

            raw_bkg = (uint8_t *) malloc(img_size + DFS747_DUMMY_PIXELS);
            if (raw_bkg == NULL) {
                status = -1;
                goto image_mode_test_error;
            }

            // Switch to image mode
            status = fps_switch_mode(dev_fd, DFS747_IMAGE_MODE, &mode_old);
            if (status < 0) {
                goto image_mode_test_error;
            }

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

            // Switch back to original mode
            status = fps_switch_mode(dev_fd, mode_old, &mode_old);
            if (status < 0) {
                goto image_mode_test_error;
            }

            memcpy(bkg_img, &raw_bkg[DFS747_DUMMY_PIXELS], img_size);

            // Free allocated buffer
            if (raw_bkg) free(raw_bkg);

            printf("\n");
            printf("    Done!\n");
            printf("\n");
        }

        if (key == '\n') {
            printf("\n");
            printf("Result:\n");
            printf("\n");
            printf("    Getting Images...\n");
            printf("\n");

            // Create an directory to save images
            time(&begin_time);
            ptm = localtime(&begin_time);
            sprintf(line, "/data/dolfa/%04d%02d%02d_%02d%02d%02d",
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
            sum = 0.0;
            for (i = 0; i < img_size; i++) {
                sum += (double) bkg_img[i];
            }
            bkg_avg = sum / img_size;

            // Switch to image mode
            status = fps_switch_mode(dev_fd, DFS747_IMAGE_MODE, &mode_old);
            if (status < 0) {
                goto image_mode_test_error;
            }

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

                // Average each frames
                for (i = 0; i < img_size; i++) {
                    sum = 0.0;
                    for (f = 0; f < avg_frame; f++) {
                        sum += (double) img_buf[f][i];
                    }

                    avg_img[i] = (uint8_t) (sum / avg_frame);
                }

                // Save (averaged) image
                sprintf(file, "%s/%03d_average.bmp", line, n);
                status = save_bmp(file, avg_img, img_size);
                if (status < 0) {
                    goto image_mode_test_error;
                }

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
                    if (avg_img[i] > bkg_img[i]) {
                        fng_img[i] = 0xFF - (avg_img[i] - bkg_img[i]);
                    } else {
                        fng_img[i] = 0xFF;
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
                if (avg_img) free(avg_img);
                for (f = 0; f < avg_frame; f++) {
                    if (raw_buf[f]) free(raw_buf[f]);
                }
                if (raw_buf) free(raw_buf);
                if (img_buf) free(img_buf);

                // Display DR
                printf("    Image %0d Dynamic Range = %0.3f\n", n, fng_dr);
            }

            status = gettimeofday(&stop_time, NULL);
            if (status < 0) {
                goto image_mode_test_error;
            }

            elapsed = get_elapsed_ms(&start_time, &stop_time);
            if (elapsed < 0) {
                goto image_mode_test_error;
            }

            // Switch back to original mode
            status = fps_switch_mode(dev_fd, mode_old, &mode_old);
            if (status < 0) {
                goto image_mode_test_error;
            }

            printf("\n");
            printf("    Done!\n");
            printf("\n");
            printf("    Time Elapsed = %0.3f ms\n", elapsed);
            printf("\n");
        }

        printf("Press ENTER key to continue... ");
        (void) getchar();
    }

    return status;

image_mode_test_error :

    // Free allocated buffers
    if (raw_bkg) free(raw_bkg);
    if (enh_img) free(enh_img);
    if (fng_img) free(fng_img);
    if (avg_img) free(avg_img);
    for (f = 0; f < avg_frame; f++) {
        if (raw_buf[f]) free(raw_buf[f]);
    }
    if (raw_buf) free(raw_buf);
    if (img_buf) free(img_buf);

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
    uint8_t  addr[9];
    uint8_t  data[9];
    uint16_t cds_offset    = 0;
    uint8_t  detect_th     = 0;
    uint8_t  row_begin     = 0;
    uint8_t  row_end       = 0;
    uint8_t  col_begin     = 0;
    uint8_t  col_end       = 0;
    uint8_t  det_width     = 0;
    uint8_t  det_height    = 0;
    uint8_t  det_window    = 0;
    uint16_t det_frame     = 1;
    int      mode_old      = 0;
    int32_t  test_limit    = -1;
    uint32_t test_cnt      = 0;
    int      test_interval = 1000;
    double   sleep_us      = 0.0;
    uint8_t  int_ctl_old   = 0;
    int      int_enable    = 1;
    int      exit_detect   = 0;
    int      detect_cnt    = 0;
    int      i;
    int st;

    while (1) {
        clear_console();

        printf("\n");
        printf("==================\n");
        printf(" Detect Mode Test \n");
        printf("==================\n");
        printf("\n");
        printf("    's' frm tst ie ex ivl - Set Detect Mode test parameters.                  \n");
        printf("                              frm = Suspend interval in frames.               \n");
        printf("                              tst = Test times in a Detect Mode test.         \n");
        printf("                                    If tst < 0, it means wait infinitely.     \n");
        printf("                              ie  = Enable detect interrupt.                  \n");
        printf("                              ex  = Exit detect mode if finger-on is detected.\n");
        printf("                              ivl = Interval in msec. between two tests.      \n");
        printf("    'g'                   - Get current Detect Mode test settings.            \n");
        printf("    ENTER                 - Do Detect Mode test based on current settings.    \n");
        printf("    'q'                   - Back to main menu.                                \n");
        printf("\n");
        printf("Pleae enter: ");

        clear_line(line, sizeof(line));
        fgets(line, sizeof(line), stdin);
        key = line[0];

        if (key == 'q') {
            break;
        }

        if (key == 's') {
            arg = strtok(line, " ");

            arg           = strtok(NULL, " ");
            det_frame     = strtol(arg, NULL, 10);
            arg           = strtok(NULL, " ");
            test_limit    = strtol(arg, NULL, 10);
            arg           = strtok(NULL, " ");
            int_enable    = strtol(arg, NULL, 10);
            arg           = strtok(NULL, " ");
            exit_detect   = strtol(arg, NULL, 10);
            arg           = strtok(NULL, " ");
            test_interval = strtol(arg, NULL, 10);

            printf("\n");
            printf("Result:\n");
            printf("\n");

            if (det_frame >= 1) {
                printf("    Suspend Interval                 = %0d frames\n", det_frame);
            } else {
                printf("    ERROR: Suspend Interval must be greater than or equal to 1!\n");
                det_frame = 1;
            }

            if (test_limit >= 1) {
                printf("    Test Times                       = %0d\n", test_limit);
            } else if (test_limit < 0) {
                printf("    Test Times                       = INFINITE\n");
            } else {
                printf("    ERROR: Test Times must be greater than or equal to 1!\n");
                test_limit = 1;
            }

            printf("    Enable Detect Interrupt          = %s\n", (int_enable  > 0) ? "TRUE" : "FALSE");
            printf("    Exit Detect Mode after Finger-On = %s\n", (exit_detect > 0) ? "TRUE" : "FALSE");

            if (test_interval >= 1) {
                printf("    Test Interval                    = %0d msec\n", test_interval);
            } else {
                printf("    ERROR: Test Interval must be greater than or equal to 1!\n");
                test_interval = 20;
            }
            printf("\n");
        }

        if (key == 'g') {
            printf("\n");
            printf("Result:\n");
            printf("\n");
            printf("    Suspend Interval                 = %0d frames\n", det_frame);
            if (test_limit < 0) {
                printf("    Test Times                       = INFINITE\n");
            } else {
                printf("    Test Times                       = %0d\n", test_limit);
            } 
            printf("    Enable Detect Interrupt          = %s\n",       (int_enable  > 0) ? "TRUE" : "FALSE");
            printf("    Exit Detect Mode after Finger-On = %s\n",       (exit_detect > 0) ? "TRUE" : "FALSE");
            printf("    Test Interval                    = %0d msec\n", test_interval);
            printf("\n");
        }

        if (key == '\n') {
            printf("\n");
            printf("Result:\n");
            printf("\n");
            printf("    Testing...\n");
            printf("\n");

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
            det_window = det_width * det_height;
            DEBUG("%s(): Detect Window = %0d (%0dx%0d)\n", __func__, det_window, det_width, det_height);

            // Get suspend interval
            det_frame = (((uint16_t) data[7]) << 8) | ((uint16_t) data[8]);
            sleep_us  = (double) (det_window * (1 + det_frame) * 10 * 8 * 2500) / 1000;
            if (sleep_us < 20.0 * MSEC) {
                sleep_us = 20.0 * MSEC;
            }

            DEBUG("%s(): Suspend Interval = %.3fus\n", __func__, sleep_us);

            // Backup original interrupt control
            status = fps_single_read(dev_fd, DFS747_REG_INT_CTL, &int_ctl_old);
            if (status < 0) {
                goto detect_mode_test_error;
            }

            // Switch to detect mode
            status = fps_switch_mode(dev_fd, DFS747_DETECT_MODE, &mode_old);
            if (status < 0) {
                goto detect_mode_test_error;
            }

            // Skip first scan
            status = fps_scan_detect_event(dev_fd, detect_th, cds_offset, sleep_us, int_enable, 0);
            if (status < 0) {
                goto detect_mode_test_error;
            }

            if (test_limit >= 1) {
                for (test_cnt = 0; test_cnt < test_limit; test_cnt++) {
                    status = fps_scan_detect_event(dev_fd, detect_th, cds_offset, sleep_us, int_enable, 0);
               
                    // Finger-on detected
                    if (status > 0) {
                        printf("    INFO: Finger-on event is detected!\n");
                        detect_cnt++;
               
                        // Exit current Detect Mode test loop
                        if (exit_detect > 0) {
                            break;
                        }
                    }
               
                    // Error
                    if (status < 0) {
                        goto detect_mode_test_error;
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
                i =0;
                st = -1;
                while (1) {
                    status = fps_scan_detect_event(dev_fd, detect_th, cds_offset, -1000, 1, 0);
                    //if(status == st)
                    //   continue;

                    st = status;
                    i++;
                    // Finger-on detected
                    if (status > 0) {
                        printf(" %d : Finger-on!\n");
                    }else
                        printf(" %d : Finger-off!\n");
                    
                    // Error
                    if (status < 0) {
                        goto detect_mode_test_error;
                    }
                }
            }

            // Switch back to original mode
            status = fps_switch_mode(dev_fd, mode_old, &mode_old);
            if (status < 0) {
                goto detect_mode_test_error;
            }

            printf("\n");
            printf("    Done!\n");
            printf("\n");
        }

        printf("Press ENTER key to continue... ");
        (void) getchar();
    }

    return status;

detect_mode_test_error :

    printf("    Failed!\n");
    printf("\n");

    return status;
}

int
powerdown_mode_test()
{
    int  status   = 0;
    char key      = 0;
    int  mode_old = 0;
    int  i;

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
        printf("Press 'q' to back to main menu ENTER to do it again: ");

        key = get_key();

        if (key == '\n') {
            continue;
        }

        if (key == 'q') {
            break;
        }
    }

    return status;

powerdown_mode_test_error :

    printf("    Failed!\n");
    printf("\n");

    return status;
}

int
reset_sensor()
{
    int      status = 0;
    char     key    = 0;
    uint32_t delay  = 10 * MSEC;

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

        status = fps_reset_sensor(dev_fd, 0);
        if (status < 0) {
            return status;
        }

        usleep(delay);

        status = fps_reset_sensor(dev_fd, 1);
        if (status < 0) {
            return status;
        }

        printf("    Done!\n");
        printf("\n");
        printf("Press 'q' to back to main menu, or ENTER to do it again: ");

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
set_debug_level()
{
    char key  = 0;
    char line[CHAR_CNT];
    char *arg = NULL;

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

        if (key == 's') {
            arg = strtok(line, " ");

            arg         = strtok(NULL, " ");
            debug_level = strtol(arg, NULL, 10);

            printf("\n");
            printf("Result:\n");
            printf("\n");
            printf("    Debug Level = %0d\n", debug_level);
            printf("\n");
        }

        if (key == 'g') {
            printf("\n");
            printf("Result:\n");
            printf("\n");
            printf("    Debug Level = %0d\n", debug_level);
            printf("\n");
        }

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

    (void) get_key();
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
    { .key = 'o', .desc = "Open the sensor",        .func = open_sensor         },
    { .key = 'c', .desc = "Close the sensor",       .func = close_sensor        },
    { .key = 'i', .desc = "Initialize the sensor",  .func = init_sensor         },
    { .key = 'r', .desc = "Read register(s)",       .func = read_register       },
    { .key = 'w', .desc = "Write register(s)",      .func = write_register      },
    { .key = 'd', .desc = "Dump all registers",     .func = dump_register       },
    { .key = 'k', .desc = "Calibrate image",        .func = calibrate_image     },
    { .key = 'K', .desc = "Calibrate detect",       .func = calibrate_detect    },
    { .key = 'I', .desc = "Image Mode test",        .func = image_mode_test     },
    { .key = 'D', .desc = "Detect Mode test",       .func = detect_mode_test    },
//  { .key = 'p', .desc = "Power-Down Mode test",   .func = powerdown_mode_test },
    { .key = 'R', .desc = "Reset the sensor",       .func = reset_sensor        },
    { .key = 'l', .desc = "Set debug level",        .func = set_debug_level     },
    { .key = 'b', .desc = "Get program build date", .func = get_build_date      },
    { .key = 'q', .desc = "Quit this program",      .func = quit_program        },
    // TODO: adding more functions here...
    { .key = 0x00, .desc = NULL, .func = NULL }
};

int
main (int argc, char *argv[])
{
    char key = 0;
    int  i;

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

