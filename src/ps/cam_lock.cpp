#include "cam_lock.h"
#include "sleep.h"
#include <stdio.h>

extern void sccb_write_reg16(u16 addr, u8 data);
extern u8   sccb_read_reg16(u16 addr);

void cam_lock_awb_aec(unsigned int settle_ms)
{
    if (settle_ms) {
        printf("cam: letting AWB/AEC settle for %u ms...\r\n", settle_ms);

        while (settle_ms > 0) {
            unsigned int chunk = (settle_ms > 500) ? 500 : settle_ms;
            usleep(chunk * 1000);
            settle_ms -= chunk;
        }
    }

    u8 exp_h  = sccb_read_reg16(0x3500);
    u8 exp_m  = sccb_read_reg16(0x3501);
    u8 exp_l  = sccb_read_reg16(0x3502);
    u8 gain_h = sccb_read_reg16(0x350a);
    u8 gain_l = sccb_read_reg16(0x350b);

    u8 awb[6];
    for (int i = 0; i < 6; i++)
        awb[i] = sccb_read_reg16(0x3400 + i);

    sccb_write_reg16(0x3500, exp_h);
    sccb_write_reg16(0x3501, exp_m);
    sccb_write_reg16(0x3502, exp_l);
    sccb_write_reg16(0x350a, gain_h);
    sccb_write_reg16(0x350b, gain_l);
    sccb_write_reg16(0x3503, 0x03);

    for (int i = 0; i < 6; i++)
        sccb_write_reg16(0x3400 + i, awb[i]);
    sccb_write_reg16(0x3406, 0x01);

    unsigned long expo = ((unsigned long)(exp_h & 0x0F) << 12) |
                         ((unsigned long)exp_m << 4) |
                         ((unsigned long)(exp_l >> 4) & 0x0F);
    unsigned int gain = ((unsigned int)(gain_h & 0x03) << 8) | gain_l;

    printf("cam: locked  exposure=%lu  gain=%u\r\n", expo, gain);
    printf("     AWB gains R=%u G=%u B=%u\r\n",
           ((unsigned int)(awb[0] & 0x0F) << 8) | awb[1],
           ((unsigned int)(awb[2] & 0x0F) << 8) | awb[3],
           ((unsigned int)(awb[4] & 0x0F) << 8) | awb[5]);

    u8 chk_aec = sccb_read_reg16(0x3503);
    u8 chk_awb = sccb_read_reg16(0x3406);
    if ((chk_aec & 0x03) != 0x03 || (chk_awb & 0x01) != 0x01) {
        printf("     !! lock FAILED (0x3503=%02x 0x3406=%02x)\r\n",
               chk_aec, chk_awb);
        printf("     detection may drift when the laser fires\r\n");
    } else {
        printf("     lock OK -- exposure/white-balance now fixed\r\n");
    }
}
