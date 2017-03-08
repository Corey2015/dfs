#ifndef __fps_control_linux_h__
#define __fps_control_linux_h__


#include <linux/types.h>

#ifdef __cplusplus
extern "C" {
#endif


////////////////////////////////////////////////////////////////////////////////
//
// Kernel Driver Interface
//

// I/O Control Opcode
#define FPS_IOC_REGISTER_MASS_READ  (0x01)
#define FPS_IOC_REGISTER_MASS_WRITE (0x02)
#define FPS_IOC_GET_ONE_IMG         (0x03)
#define FPS_IOC_READ_CHIP_ID        (0x04)
#define FPS_IOC_RESET_SENSOR        (0x07)
#define FPS_IOC_INTR_INIT           (0xA4)
#define FPS_IOC_INTR_CLOSE          (0xA5)
#define FPS_IOC_INTR_READ           (0xA6)

struct fps_ioc_transfer {
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

#define FPS_IOC_MAGIC ('k')
#define FPS_MSGSIZE(N) \
    ((((N) * (sizeof (struct fps_ioc_transfer))) < (1 << _IOC_SIZEBITS)) ? \
     ((N) * (sizeof (struct fps_ioc_transfer))) : 0)
#define FPS_IOC_MESSAGE(N) _IOW(FPS_IOC_MAGIC, 0, char[FPS_MSGSIZE(N)])


#ifdef __cplusplus
}
#endif


#endif // __fps_control_linux_h__
