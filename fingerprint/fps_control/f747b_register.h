#ifndef __f747b_register_h__
#define __f747b_register_h__


////////////////////////////////////////////////////////////////////////////////
//
// Sensor Dimension
//

#define FPS_SENSOR_COLS  (144)
#define FPS_SENSOR_ROWS  (64)
#define FPS_DUMMY_PIXELS (1)


////////////////////////////////////////////////////////////////////////////////
//
// Sensor Commands
//

#define FPS_CMD_WR_REG       (0xD0)
#define FPS_CMD_BURST_WR_REG (0xD8)
#define FPS_CMD_RD_REG       (0xD4)
#define FPS_CMD_BURST_RD_REG (0xDC)
#define FPS_CMD_RD_CHIP_ID   (0xDD)
#define FPS_CMD_BURST_RD_IMG (0xDE)
#define DUMMY_DATA             (0xFF)


////////////////////////////////////////////////////////////////////////////////
//
// Control Registers
//

#define FPS_REG_COUNT             (0x3C)
#define FPS_REG_INT_EVENT         (0x00)
#define FPS_REG_INT_CTL           (0x01)
    #define FPS_DETECT_EVENT          (1 << 3)
    #define FPS_FRAME_READY_EVENT     (1 << 2)
    #define FPS_RESET_EVENT           (1 << 0)
    #define FPS_ALL_EVENTS            (0xFF)
#define FPS_REG_GBL_CTL           (0x02)
    #define FPS_ENABLE_DETECT         (1 << 4)
    #define FPS_ENABLE_PWRDWN         (1 << 0)
#define FPS_REG_PWR_CTL_0         (0x03)
    #define FPS_PWRDWN_DET            (1 << 7)
    #define FPS_PWRDWN_PGA            (1 << 6)
    #define FPS_PWRDWN_PGA_BUF        (1 << 5)
    #define FPS_PWRDWN_BGR            (1 << 4)
    #define FPS_PWRDWN_V2I            (1 << 3)
    #define FPS_PWRDWN_OSC            (1 << 2)
    #define FPS_PWRDWN_FPS            (1 << 1)
    #define FPS_PWRDWN_ADC            (1 << 0)
    #define FPS_PWRDWN_ALL            ((0xFF) & ~FPS_PWRDWN_BGR)  // NOTE: Workaround!
#define FPS_REG_MISC_PWR_CTL_1    (0x04)
    #define FPS_PWRDWN_OVT            (1 << 0)
#define FPS_REG_CPR_CTL_0         (0x05)
#define FPS_REG_CPR_CTL_1         (0x06)
#define FPS_REG_OVT_TEST_CTL      (0x07)
#define FPS_REG_ANA_I_SET_0       (0x08)
#define FPS_REG_ANA_I_SET_1       (0x09)
#define FPS_REG_SUSP_WAIT_F_CYC_H (0x0A)
#define FPS_REG_SUSP_WAIT_F_CYC_L (0x0B)
#define FPS_REG_IMG_CDS_CTL_0     (0x0C)
    #define FPS_MIN_CDS_OFFSET_0      (0x00)
    #define FPS_MAX_CDS_OFFSET_0      (0x7F)
#define FPS_REG_IMG_CDS_CTL_1     (0x0D)
    #define FPS_MIN_CDS_OFFSET_1      (0x0000)
    #define FPS_MAX_CDS_OFFSET_1      (0x01FF)
#define FPS_REG_IMG_PGA0_CTL      (0x0E)
    #define FPS_MIN_PGA_GAIN_0        (0x00)
    #define FPS_MAX_PGA_GAIN_0        (0x0F)
#define FPS_REG_IMG_PGA1_CTL      (0x0F)
    #define FPS_MIN_PGA_GAIN_1        (0x00)
    #define FPS_MAX_PGA_GAIN_1        (0x0F)
#define FPS_REG_IMG_ROW_BEGIN     (0x10)
#define FPS_REG_IMG_ROW_END       (0x11)
#define FPS_REG_IMG_COL_BEGIN     (0x12)
#define FPS_REG_IMG_COL_END       (0x13)
#define FPS_REG_DET_CDS_CTL_0     (0x14)
#define FPS_REG_DET_CDS_CTL_1     (0x15)
#define FPS_REG_DET_PGA0_CTL      (0x16)
#define FPS_REG_DET_PGA1_CTL      (0x17)
#define FPS_REG_DET_ROW_BEGIN     (0x18)
#define FPS_REG_DET_ROW_END       (0x19)
#define FPS_REG_DET_COL_BEGIN     (0x1A)
#define FPS_REG_DET_COL_END       (0x1B)
#define FPS_REG_V_DET_SEL         (0x1C)
    #define FPS_MAX_DETECT_TH         (0x3F)
    #define FPS_MIN_DETECT_TH         (0x00)
#define FPS_REG_TEST_ANA          (0x1D)
#define FPS_REG_SET_ANA_0         (0x1E)
#define FPS_REG_SET_ANA_1         (0x1F)
#define FPS_REG_SET_ANA_2         (0x20)


#endif // __f747b_register_h__
