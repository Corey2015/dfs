#ifndef __FPS_DRIVER_H__
#define __FPS_DRIVER_H__


#include <asm/ioctl.h>
#include <linux/types.h>


////////////////////////////////////////////////////////////////////////////////
//
// Constants
//

#define CHAR_CNT (300)
#define USEC     (1)
#define MSEC     (1000 * USEC)
#define SEC      (1000 * MSEC)


////////////////////////////////////////////////////////////////////////////////
//
// Debug
//

#define ERROR(_fmt_, _arg_...)  do { if(debug_level >= 0) printf(" [ ERROR ] " _fmt_, ## _arg_); } while(0)
#define WARN(_fmt_, _arg_...)   do { if(debug_level >= 1) printf("  [ WARN ] " _fmt_, ## _arg_); } while(0)
#define INFO(_fmt_, _arg_...)   do { if(debug_level >= 2) printf("  [ INFO ] " _fmt_, ## _arg_); } while(0)
#define DEBUG(_fmt_, _arg_...)  do { if(debug_level >= 3) printf(" [ DEBUG ] " _fmt_, ## _arg_); } while(0)
#define DETAIL(_fmt_, _arg_...) do { if(debug_level >= 4) printf("[ DETAIL ] " _fmt_, ## _arg_); } while(0)


////////////////////////////////////////////////////////////////////////////////
//
// Driver Related Definitions
//

// I/O Control Opcode
#define DFS747_IOC_REGISTER_MASS_READ  (0x01)
#define DFS747_IOC_REGISTER_MASS_WRITE (0x02)
#define DFS747_IOC_GET_ONE_IMG         (0x03)
#define DFS747_IOC_RESET_SENSOR        (0x07)
#define DFS747_IOC_SET_CLKRATE         (0x08)
#define DFS747_IOC_WAKELOCK            (0x09)
#define DFS747_IOC_SENDKEY             (0x10)
#define DFS747_IOC_INTR_INIT           (0xA4)
#define DFS747_IOC_INTR_CLOSE          (0xA5)
#define DFS747_IOC_INTR_READ           (0xA6)

struct dfs747_ioc_transfer {
    __u64 tx_buf;
    __u64 rx_buf;
    __u32 len;
    __u32 speed_hz;
    __u16 delay_usecs;
    __u8  bits_per_word;
    __u8  cs_change;
    __u8  opcode;
    __u8  pad[3];
};

#define DFS747_IOC_MAGIC ('k')
#define DFS747_MSGSIZE(N) \
    ((((N) * (sizeof (struct dfs747_ioc_transfer))) < (1 << _IOC_SIZEBITS)) ? \
     ((N) * (sizeof (struct dfs747_ioc_transfer))) : 0)
#define DFS747_IOC_MESSAGE(N) _IOW(DFS747_IOC_MAGIC, 0, char[DFS747_MSGSIZE(N)])


////////////////////////////////////////////////////////////////////////////////
//
// Sensor Related Definitions
//

// Pin Assignment
#define DFS747_RESET_PIN             GPIO_FIGERPRINT_RST         // Reset pin
#define DFS747_POWER_PIN             GPIO_FIGERPRINT_PWR_EN_PIN  // Power Pin
#if defined(GPIO_FIGERPRINT_PWR_EN2_PIN)
#define DFS747_POWER2_PIN            GPIO_FIGERPRINT_PWR_EN2_PIN // Power Pin for 1.8V
#endif

// Sensor Dimension
#define DFS747_SENSOR_ROWS           (64)
#define DFS747_SENSOR_COLS           (144)
#define DFS747_SENSOR_SIZE           (DFS747_SENSOR_ROWS * DFS747_SENSOR_COLS)
#define DFS747_DUMMY_PIXELS          (1)

// SPI Commands
#define DFS747_CMD_WR_REG            (0xD0)
#define DFS747_CMD_BURST_WR_REG      (0xD8)
#define DFS747_CMD_RD_REG            (0xD4)
#define DFS747_CMD_BURST_RD_REG      (0xDC)
#define DFS747_CMD_RD_CHIP_ID        (0xDD)
#define DFS747_CMD_BURST_RD_IMG      (0xDE)
#define DUMMY_DATA                    (0xFF)

// Control Registers
#define DFS747_REG_COUNT             (0x3C)
#define DFS747_REG_INT_EVENT         (0x00)
#define DFS747_REG_INT_CTL           (0x01)
    #define DFS747_DETECT_EVENT          (1 << 3)
    #define DFS747_FRAME_READY_EVENT     (1 << 2)
    #define DFS747_RESET_EVENT           (1 << 0)
#define DFS747_REG_GBL_CTL           (0x02)
    #define DFS747_ENABLE_DETECT         (1 << 4)
    #define DFS747_ENABLE_PWRDWN         (1 << 0)
#define DFS747_REG_PWR_CTL_0         (0x03)
    #define DFS747_PWRDWN_DET            (1 << 7)
    #define DFS747_PWRDWN_PGA            (1 << 6)
    #define DFS747_PWRDWN_PGA_BUF        (1 << 5)
    #define DFS747_PWRDWN_BGR            (1 << 4)
    #define DFS747_PWRDWN_V2I            (1 << 3)
    #define DFS747_PWRDWN_OSC            (1 << 2)
    #define DFS747_PWRDWN_FPS            (1 << 1)
    #define DFS747_PWRDWN_ADC            (1 << 0)
    #define DFS747_PWRDWN_ALL            (0xFF)
#define DFS747_REG_MISC_PWR_CTL_1    (0x04)
    #define DFS747_PWRDWN_OVT            (1 << 0)
#define DFS747_REG_CPR_CTL_0         (0x05)
#define DFS747_REG_CPR_CTL_1         (0x06)
#define DFS747_REG_SUSP_WAIT_F_CYC_H (0x0A)
#define DFS747_REG_SUSP_WAIT_F_CYC_L (0x0B)
#define DFS747_REG_IMG_CDS_CTL_0     (0x0C)
#define DFS747_REG_IMG_CDS_CTL_1     (0x0D)
#define DFS747_REG_IMG_PGA0_CTL      (0x0E)
#define DFS747_REG_IMG_PGA1_CTL      (0x0F)
#define DFS747_REG_IMG_ROW_BEGIN     (0x10)
#define DFS747_REG_IMG_ROW_END       (0x11)
#define DFS747_REG_IMG_COL_BEGIN     (0x12)
#define DFS747_REG_IMG_COL_END       (0x13)
#define DFS747_REG_DET_CDS_CTL_0     (0x14)
#define DFS747_REG_DET_CDS_CTL_1     (0x15)
#define DFS747_REG_DET_PGA0_CTL      (0x16)
#define DFS747_REG_DET_PGA1_CTL      (0x17)
#define DFS747_REG_DET_ROW_BEGIN     (0x18)
#define DFS747_REG_DET_ROW_END       (0x19)
#define DFS747_REG_DET_COL_BEGIN     (0x1A)
#define DFS747_REG_DET_COL_END       (0x1B)
#define DFS747_REG_V_DET_SEL         (0x1C)
#define DFS747_REG_TEST_ANA          (0x1D)

#define DFS747_MAX_DETECT_TH         (0x3F)
#define DFS747_MIN_DETECT_TH         (0x00)
#define DFS747_MAX_CDS_OFFSET        (0x01FF)
#define DFS747_MIN_CDS_OFFSET        (0x0000)
#define DFS747_MAX_PGA_GAIN          (0x0F)
#define DFS747_MIN_PGA_GAIN          (0x00)

// Power Modes
#define DFS747_IMAGE_MODE            (0)
#define DFS747_DETECT_MODE           (1)
#define DFS747_POWER_DOWN_MODE       (2)



#endif // __FPS_DRIVER_H__
