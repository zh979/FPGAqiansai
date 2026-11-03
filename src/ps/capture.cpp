#pragma GCC optimize(3, "inline")

#include "capture.h"
#include "yolo_pre.h"
#include "sd_io.h"
#include "xil_cache.h"
#include "xuartps_hw.h"
#include "xparameters.h"
#include "xstatus.h"
#include <stdio.h>
#include <string.h>

extern int SD_Transfer_write(char* FileName, u32 SourceAddress, u32 ByteLength);

#define UART_BASE   XPAR_PS7_UART_0_BASEADDR

#define CAP_MAX_W   1280
#define CAP_MAX_H   800
#define BMP_HDR     54
static unsigned char g_bmp[BMP_HDR + CAP_MAX_W * CAP_MAX_H * 3];

static void put_u32(unsigned char* p, unsigned int v)
{
    p[0] = (unsigned char)(v & 0xFF);
    p[1] = (unsigned char)((v >> 8) & 0xFF);
    p[2] = (unsigned char)((v >> 16) & 0xFF);
    p[3] = (unsigned char)((v >> 24) & 0xFF);
}

static unsigned int pack_bmp(const unsigned char* fb, int w, int h, int stride)
{
    const int row_raw = w * 3;
    const int pad = (4 - (row_raw & 3)) & 3;
    const int row_bmp = row_raw + pad;
    const unsigned int img_bytes = (unsigned int)row_bmp * (unsigned int)h;
    const unsigned int total = BMP_HDR + img_bytes;

    unsigned char* p = g_bmp;
    memset(p, 0, BMP_HDR);
    p[0] = 'B'; p[1] = 'M';
    put_u32(p + 2, total);
    put_u32(p + 10, BMP_HDR);
    put_u32(p + 14, 40);
    put_u32(p + 18, (unsigned int)w);
    put_u32(p + 22, (unsigned int)h);
    p[26] = 1;
    p[28] = 24;
    put_u32(p + 34, img_bytes);

#if YPRE_SRC_BGR
    const int O_R = 2, O_G = 1, O_B = 0;
#else
    const int O_R = 0, O_G = 1, O_B = 2;
#endif

    unsigned char* dst = g_bmp + BMP_HDR;
    for (int y = 0; y < h; y++) {
        const unsigned char* src = fb + (long)(h - 1 - y) * stride;
        unsigned char* d = dst + (long)y * row_bmp;
        for (int x = 0; x < w; x++) {
            d[0] = src[O_B];
            d[1] = src[O_G];
            d[2] = src[O_R];
            d += 3;
            src += 3;
        }
        for (int k = 0; k < pad; k++)
            d[k] = 0;
    }
    return total;
}

static int uart_getc_nb(void)
{
    if (XUartPs_IsReceiveData(UART_BASE))
        return (int)XUartPs_RecvByte(UART_BASE);
    return 0;
}

static void make_path(char* buf, const char* dir, int idx, int scene)
{
    int i = 0;
    while (dir[i] && i < 60) { buf[i] = dir[i]; i++; }
    buf[i++] = '\\';
    buf[i++] = 's'; buf[i++] = 'h'; buf[i++] = 'o'; buf[i++] = 't'; buf[i++] = '_';

    buf[i++] = (char)('0' + (idx / 1000) % 10);
    buf[i++] = (char)('0' + (idx / 100) % 10);
    buf[i++] = (char)('0' + (idx / 10) % 10);
    buf[i++] = (char)('0' + idx % 10);
    if (scene > 0) {
        buf[i++] = '_'; buf[i++] = 's';
        buf[i++] = (char)('0' + scene);
    }
    buf[i++] = '.'; buf[i++] = 'b'; buf[i++] = 'm'; buf[i++] = 'p';
    buf[i] = 0;
}

static int scan_next_index(const char* dir)
{
    static FILINFO fno;
    DIR dp;
    int maxi = -1, n_seen = 0;

    if (f_opendir(&dp, dir) != FR_OK) {
        printf("capture: cannot scan %s, starting at 0\r\n", dir);
        return 0;
    }
    while (f_readdir(&dp, &fno) == FR_OK && fno.fname[0]) {
        if (fno.fattrib & AM_DIR)
            continue;
        if (strncmp(fno.fname, "shot_", 5) != 0)
            continue;

        int v = 0, k;
        for (k = 0; k < 4; k++) {
            char c = fno.fname[5 + k];
            if (c < '0' || c > '9') { v = -1; break; }
            v = v * 10 + (c - '0');
        }
        if (v < 0)
            continue;
        n_seen++;
        if (v > maxi)
            maxi = v;
    }
    f_closedir(&dp);

    if (n_seen)
        printf("capture: found %d existing shots, max index %04d\r\n",
               n_seen, maxi);
    return maxi + 1;
}

void capture_mode(unsigned int fb_cap, int w, int h, const char* out_dir)
{
    if (w > CAP_MAX_W || h > CAP_MAX_H) {
        printf("capture: %dx%d exceeds buffer %dx%d\r\n",
               w, h, CAP_MAX_W, CAP_MAX_H);
        return;
    }

    const int stride = w * 3;
    const unsigned int fb_bytes = (unsigned int)stride * (unsigned int)h;

    printf("\r\n=== Dataset capture mode ===\r\n");
    printf("resolution %dx%d, output dir: %s\r\n", w, h, out_dir);
    printf("the SD card must already contain this directory\r\n");
    printf("\r\n");
    printf("keys:\r\n");
    printf("  SPACE / ENTER  capture one frame\r\n");
    printf("  1..5           capture and tag with scene number\r\n");
    printf("                 1 = normal light   2 = dim / lights off\r\n");
    printf("                 3 = side lamp      4 = belt moving\r\n");
    printf("                 5 = weed touching / overlapping crop\r\n");
    printf("  s              show counters\r\n");
    printf("  q              quit capture mode\r\n");
    printf("\r\n");
    printf("tip: keep camera height and lighting fixed; vary target\r\n");
    printf("     position, count, orientation and combination instead.\r\n");
    printf("\r\n");

    int idx = scan_next_index(out_dir);
    const int idx_begin = idx;
    printf("starting at index %04d\r\n\r\n", idx);

    int scene_cnt[6] = {0, 0, 0, 0, 0, 0};
    char path[96];

    while (1) {
        int c = uart_getc_nb();
        if (c == 0)
            continue;

        if (c == 'q' || c == 'Q') {
            printf("capture: quit, %d saved this session (%d on card)\r\n",
                   idx - idx_begin, idx);
            break;
        }

        if (c == 's' || c == 'S') {
            printf("this session %d, card total %d:", idx - idx_begin, idx);
            for (int k = 0; k <= 5; k++)
                if (scene_cnt[k])
                    printf("  s%d=%d", k, scene_cnt[k]);
            printf("\r\n");
            continue;
        }

        int scene = 0;
        if (c >= '1' && c <= '5')
            scene = c - '0';
        else if (c != ' ' && c != '\r' && c != '\n')
            continue;

        Xil_DCacheInvalidateRange(fb_cap, fb_bytes);
        unsigned int n = pack_bmp((const unsigned char*)fb_cap, w, h, stride);

        make_path(path, out_dir, idx, scene);

        if (SD_Transfer_write(path, (u32)(unsigned long)g_bmp, n) == XST_SUCCESS) {
            scene_cnt[scene]++;
            idx++;
            printf("saved %s  (%u bytes, session %d, card %d)\r\n",
                   path, n, idx - idx_begin, idx);
        } else {
            printf("!! write failed: %s -- is the directory present?\r\n", path);
        }
    }
}
