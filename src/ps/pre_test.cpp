#pragma GCC optimize(3, "inline")

#include "yolo_pre.h"
#include "sd_io.h"
#include "xtime_l.h"
#include "xil_cache.h"
#include <stdio.h>

static data_t g_net_in[3 * YPRE_NET_SIZE * YPRE_NET_SIZE];

void pre_selftest(unsigned int fb_addr, int fb_w, int fb_h, const char* out_dir)
{
    const unsigned char* fb = (const unsigned char*)fb_addr;

    Xil_DCacheInvalidateRange((unsigned int)fb_addr,
                              (unsigned int)(fb_w * fb_h * 3));

    printf("\r\n=== Preprocess self-test ===\r\n");
    printf("framebuffer 0x%08X, %dx%d, YPRE_SRC_BGR=%d\r\n",
           fb_addr, fb_w, fb_h, YPRE_SRC_BGR);

    const int modes[3]  = { YPRE_NEAREST, YPRE_BOX2X2, YPRE_BILINEAR };
    const char* names[3] = { "nearest ", "box2x2  ", "bilinear" };

    for (int m = 0; m < 3; m++) {
        XTime t0, t1;
        XTime_GetTime(&t0);
        int rc = yolo_preprocess(fb, fb_w, fb_h, 0,
                                 0, 0, fb_w, fb_h,
                                 modes[m], g_net_in);
        XTime_GetTime(&t1);

        unsigned long us = (unsigned long)(((t1 - t0) * 1000000) / COUNTS_PER_SECOND);
        printf("  %s  rc=%d  %lu us\r\n", names[m], rc, us);
    }

    long sum[3] = {0, 0, 0};
    const int npx = YPRE_NET_SIZE * YPRE_NET_SIZE;
    for (int ch = 0; ch < 3; ch++) {
        for (int i = 0; i < npx; i++)
            sum[ch] += g_net_in[ch * npx + i];
    }
    printf("channel mean (Q7.9, full=512): ch0=%ld ch1=%ld ch2=%ld\r\n",
           sum[0] / npx, sum[1] / npx, sum[2] / npx);

    if (sum[0] == 0 && sum[1] == 0 && sum[2] == 0)
        printf("  !! all zero -- check framebuffer addr / camera output\r\n");
    else if (sum[0] / npx > 240 && sum[0] / npx < 275 &&
             sum[1] / npx > 240 && sum[1] / npx < 275 &&
             sum[2] / npx > 240 && sum[2] / npx < 275)
        printf("  !! all near 256 -- looks like uninitialized DDR, not a real frame\r\n");

    {
        char path[128];
        int i = 0;
        while (out_dir[i] && i < 100) { path[i] = out_dir[i]; i++; }
        const char* tail = "\\pre.bin";
        int j = 0;
        while (tail[j]) { path[i++] = tail[j++]; }
        path[i] = 0;

        write_data(string(path), g_net_in, 3 * npx);
        printf("wrote %s\r\n", path);
        printf("On PC: python pre_check.py view pre.bin out.png f\r\n");
    }
}
