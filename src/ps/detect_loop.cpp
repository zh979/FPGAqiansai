#pragma GCC optimize(3, "inline")

#include "yolo_pre.h"
#include "yolo_post.h"
#include "motion.h"
#include "gimbal.h"
#include "laser.h"
#include "yolo4_tiny.h"
#include "xtime_l.h"
#include "xil_cache.h"
#include "sleep.h"
#include "xuartps_hw.h"
#include "xstatus.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define CLEAN_INPUT      1

#define CAM_FRAME_US     40000

#define CLEAR_CONFIRM    3

#define IDLE_PROBE       50

#define LOG_CAP   (192 * 1024)
static char g_log[LOG_CAP];
static int  g_log_n = 0;
static int  g_log_full = 0;

extern int SD_Transfer_write(char* FileName, u32 SourceAddress, u32 ByteLength);

static void logf_both(const char* fmt, ...)
{
    char line[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(line) - 1) n = (int)sizeof(line) - 1;

    fputs(line, stdout);

    if (!g_log_full) {
        if (g_log_n + n < LOG_CAP) {
            memcpy(g_log + g_log_n, line, n);
            g_log_n += n;
        } else {
            g_log_full = 1;

            printf("\r\n!! LOG BUFFER FULL at tick -- press q NOW to save, "
                   "later data will be LOST\r\n");
        }
    } else {
        static int warn_cnt = 0;
        if ((++warn_cnt & 63) == 0)
            printf("!! log full, data since then is not being recorded\r\n");
    }
}

static void log_flush(const char* fname)
{
    if (g_log_n <= 0) {
        printf("log: nothing to write\r\n");
        return;
    }
    if (SD_Transfer_write((char*)fname, (u32)(unsigned long)g_log,
                          (u32)g_log_n) == XST_SUCCESS)
        printf("log: wrote %s (%d bytes%s)\r\n", fname, g_log_n,
               g_log_full ? ", TRUNCATED" : "");
    else
        printf("log: could not write %s\r\n", fname);
}

#if (YP_OUT_CH != OUT_CH)
#error "类别数不匹配: yolo_post.h 的 YP_MODEL_WEED 与 basic_op.h 的 OUT_CH 要配套 (2类->21, 80类->255)"
#endif

static data_t g_net_in[3 * YPRE_NET_SIZE * YPRE_NET_SIZE];
static data_t g_out0[YP_OUT_CH * YP_GRID0 * YP_GRID0];
static data_t g_out1[YP_OUT_CH * YP_GRID1 * YP_GRID1];
static YoloBox g_dets[YP_MAX_DET];

static void draw_rect(unsigned char* fb, int w, int h, int stride,
                      int x0, int y0, int x1, int y1,
                      unsigned char r, unsigned char g, unsigned char b,
                      int thick)
{
    if (x0 > x1) { int t = x0; x0 = x1; x1 = t; }
    if (y0 > y1) { int t = y0; y0 = y1; y1 = t; }
    if (x1 < 0 || y1 < 0 || x0 >= w || y0 >= h)
        return;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= w) x1 = w - 1;
    if (y1 >= h) y1 = h - 1;

#if YPRE_SRC_BGR
    const int O_R = 2, O_G = 1, O_B = 0;
#else
    const int O_R = 0, O_G = 1, O_B = 2;
#endif

    for (int t = 0; t < thick; t++) {
        int yt = y0 + t, yb = y1 - t;
        if (yt <= yb) {
            unsigned char* pt = fb + (long)yt * stride + (long)x0 * 3;
            unsigned char* pb = fb + (long)yb * stride + (long)x0 * 3;
            for (int x = x0; x <= x1; x++) {
                pt[O_R] = r; pt[O_G] = g; pt[O_B] = b; pt += 3;
                pb[O_R] = r; pb[O_G] = g; pb[O_B] = b; pb += 3;
            }
        }
        int xl = x0 + t, xr = x1 - t;
        if (xl <= xr) {
            for (int y = y0; y <= y1; y++) {
                unsigned char* pl = fb + (long)y * stride + (long)xl * 3;
                unsigned char* pr = fb + (long)y * stride + (long)xr * 3;
                pl[O_R] = r; pl[O_G] = g; pl[O_B] = b;
                pr[O_R] = r; pr[O_G] = g; pr[O_B] = b;
            }
        }
    }
}

static void fill_rect(unsigned char* fb, int w, int h, int stride,
                      int x0, int y0, int x1, int y1,
                      unsigned char r, unsigned char g, unsigned char b)
{
    if (x0 > x1) { int t = x0; x0 = x1; x1 = t; }
    if (y0 > y1) { int t = y0; y0 = y1; y1 = t; }
    if (x1 < 0 || y1 < 0 || x0 >= w || y0 >= h)
        return;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= w) x1 = w - 1;
    if (y1 >= h) y1 = h - 1;

#if YPRE_SRC_BGR
    const int O_R = 2, O_G = 1, O_B = 0;
#else
    const int O_R = 0, O_G = 1, O_B = 2;
#endif

    for (int y = y0; y <= y1; y++) {
        unsigned char* p = fb + (long)y * stride + (long)x0 * 3;
        for (int x = x0; x <= x1; x++) {
            p[O_R] = r; p[O_G] = g; p[O_B] = b; p += 3;
        }
    }
}

static void class_color(int cid, unsigned char* r, unsigned char* g,
                        unsigned char* b)
{
    if (cid == YP_CLS_WEED) {
        *r = 0;   *g = 255; *b = 0;
    } else if (cid == YP_CLS_CROP) {
        *r = 0;   *g = 160; *b = 255;
    } else {
        *r = (unsigned char)(80 + (cid * 97) % 176);
        *g = (unsigned char)(80 + (cid * 43) % 176);
        *b = (unsigned char)(80 + (cid * 61) % 176);
    }
}

extern double g_bench_ms_pl;

static struct {
    unsigned int fb;
    int w, h, stride;
    int x0, y0;
    int bar_w, bar_h;

    int   active;
    float frac[2];
    int   done[2];
} g_bd;

static void bench_repaint(void)
{
    if (!g_bd.active)
        return;

    static XTime s_last = 0;
    XTime now;
    XTime_GetTime(&now);
    if (s_last != 0 &&
        now - s_last < (unsigned long long)(COUNTS_PER_SECOND / 60))
        return;
    s_last = now;

    unsigned char* fb = (unsigned char*)g_bd.fb;

    for (int row = 0; row < 2; row++) {
        const int y = g_bd.y0 + row * (g_bd.bar_h + 14);
        const int x = g_bd.x0;

        fill_rect(fb, g_bd.w, g_bd.h, g_bd.stride,
                  x - 2, y - 2, x + g_bd.bar_w + 2, y + g_bd.bar_h + 2,
                  40, 40, 40);

        float len_frac = g_bd.frac[row];
        if (len_frac < 0.0f) len_frac = 0.0f;
        if (len_frac > 1.0f) len_frac = 1.0f;

        int len = (int)(len_frac * g_bd.bar_w);

        unsigned char r, g, b;
        if (row == 0) {
            r = 0; g = 230; b = 90;
        } else {
            r = 255; g = g_bd.done[1] ? 80 : 160; b = 60;
        }

        if (len > 0)
            fill_rect(fb, g_bd.w, g_bd.h, g_bd.stride,
                      x, y, x + len, y + g_bd.bar_h, r, g, b);

        if (g_bd.done[row] && len > 6)
            fill_rect(fb, g_bd.w, g_bd.h, g_bd.stride,
                      x + len - 6, y, x + len, y + g_bd.bar_h,
                      255, 255, 255);
    }

    int fy0 = g_bd.y0 - 4;
    int fy1 = g_bd.y0 + 2 * (g_bd.bar_h + 14) + 4;
    if (fy0 < 0) fy0 = 0;
    if (fy1 > g_bd.h) fy1 = g_bd.h;
    Xil_DCacheFlushRange(g_bd.fb + (unsigned int)fy0 * g_bd.stride,
                         (unsigned int)(fy1 - fy0) * g_bd.stride);
}

static void bench_draw(int row, float frac, int done)
{
    if (row >= 0 && row < 2) {
        g_bd.frac[row] = frac;
        g_bd.done[row] = done;
    }
    bench_repaint();
}

static struct {
    unsigned int fb;
    int w, h, stride;
    YoloBox* dets;
    int nd;
    XTime last;
    unsigned long long period;
    unsigned long count;

    Gimbal* gb;
    XTime ka_last;
    unsigned long long ka_period;

    int fire_on;
    float fire_x, fire_y;
    float fire_w, fire_h;

    int   hud_on;
    int   hud_h;
    int   duty;
} g_rp;

static const unsigned char SEG7[10] = {
    0x3F,
    0x06,
    0x5B,
    0x4F,
    0x66,
    0x6D,
    0x7D,
    0x07,
    0x7F,
    0x6F,
};

static void draw_digit(unsigned char* fb, int fw, int fh, int stride,
                       int x, int y, int dw, int dh, int t, int d,
                       unsigned char r, unsigned char g, unsigned char b)
{
    if (d < 0 || d > 9)
        return;
    const unsigned char s = SEG7[d];
    const int mid = y + dh / 2;

    if (s & 0x01) fill_rect(fb, fw, fh, stride, x, y, x + dw, y + t, r, g, b);
    if (s & 0x02) fill_rect(fb, fw, fh, stride, x + dw - t, y, x + dw, mid, r, g, b);
    if (s & 0x04) fill_rect(fb, fw, fh, stride, x + dw - t, mid, x + dw, y + dh, r, g, b);
    if (s & 0x08) fill_rect(fb, fw, fh, stride, x, y + dh - t, x + dw, y + dh, r, g, b);
    if (s & 0x10) fill_rect(fb, fw, fh, stride, x, mid, x + t, y + dh, r, g, b);
    if (s & 0x20) fill_rect(fb, fw, fh, stride, x, y, x + t, mid, r, g, b);
    if (s & 0x40) fill_rect(fb, fw, fh, stride, x, mid - t / 2, x + dw, mid + t - t / 2, r, g, b);
}

static void draw_percent(unsigned char* fb, int fw, int fh, int stride,
                         int x, int y, int dw, int dh, int t,
                         unsigned char r, unsigned char g, unsigned char b)
{
    int s = t + 1;
    fill_rect(fb, fw, fh, stride, x, y, x + s, y + s, r, g, b);
    fill_rect(fb, fw, fh, stride, x + dw - s, y + dh - s, x + dw, y + dh, r, g, b);
    for (int i = 0; i < dh; i++) {
        int px = x + dw - 1 - (i * (dw - 1)) / (dh - 1);
        fill_rect(fb, fw, fh, stride, px, y + i, px + t - 1, y + i, r, g, b);
    }
}

static void draw_hud(void)
{
    if (!g_rp.hud_on)
        return;

    static XTime s_last = 0;
    XTime now;
    XTime_GetTime(&now);
    if (s_last != 0 && now - s_last < (unsigned long long)(COUNTS_PER_SECOND / 60))
        return;
    s_last = now;

    unsigned char* fb = (unsigned char*)g_rp.fb;
    const int H  = g_rp.hud_h;
    const int y0 = g_rp.h - H;
    const int w  = g_rp.w;

    fill_rect(fb, w, g_rp.h, g_rp.stride, 0, y0, w - 1, g_rp.h - 1,
              18, 18, 22);

    int d = g_rp.duty;
    if (d < 0) d = 0;
    if (d > 100) d = 100;

    const int pad = 10;
    const int dh  = H - 12;
    const int dw  = dh * 3 / 5;
    const int t   = (dh >= 24) ? 4 : 3;
    const int gap = 5;
    const int ty  = y0 + 6;

    unsigned char nr = 255, ng = 255, nb = 255;

    int dx = pad;
    if (d >= 100) {
        draw_digit(fb, w, g_rp.h, g_rp.stride, dx, ty, dw, dh, t, 1, nr, ng, nb);
        dx += dw + gap;
        draw_digit(fb, w, g_rp.h, g_rp.stride, dx, ty, dw, dh, t, 0, nr, ng, nb);
        dx += dw + gap;
        draw_digit(fb, w, g_rp.h, g_rp.stride, dx, ty, dw, dh, t, 0, nr, ng, nb);
        dx += dw + gap;
    } else {
        dx += dw + gap;
        if (d >= 10) {
            draw_digit(fb, w, g_rp.h, g_rp.stride, dx, ty, dw, dh, t,
                       d / 10, nr, ng, nb);
        }
        dx += dw + gap;
        draw_digit(fb, w, g_rp.h, g_rp.stride, dx, ty, dw, dh, t,
                   d % 10, nr, ng, nb);
        dx += dw + gap;
    }
    draw_percent(fb, w, g_rp.h, g_rp.stride, dx, ty, dw, dh, t, nr, ng, nb);
    dx += dw + gap * 3;

    const int bar_x = dx;
    const int bar_w = w - bar_x - pad;
    const int bar_y = y0 + 8;
    const int bar_h = H - 16;

    fill_rect(fb, w, g_rp.h, g_rp.stride,
              bar_x, bar_y, bar_x + bar_w, bar_y + bar_h, 48, 48, 54);

    unsigned char dr, dg, db;
    if (d > 60)      { dr = 255; dg = 140; db = 40; }
    else if (d > 25) { dr = 230; dg = 210; db = 50; }
    else             { dr = 60;  dg = 220; db = 90; }

    int dl = bar_w * d / 100;
    if (dl > 0)
        fill_rect(fb, w, g_rp.h, g_rp.stride,
                  bar_x, bar_y, bar_x + dl, bar_y + bar_h, dr, dg, db);

    for (int i = 1; i <= 3; i++) {
        int tx = bar_x + bar_w * i / 4;
        fill_rect(fb, w, g_rp.h, g_rp.stride,
                  tx, bar_y, tx, bar_y + bar_h, 105, 105, 115);
    }

    Xil_DCacheFlushRange(g_rp.fb + (unsigned int)y0 * g_rp.stride,
                         (unsigned int)H * g_rp.stride);
}

static void repaint_hook(void)
{
    XTime now;
    XTime_GetTime(&now);

    if (g_rp.gb && now - g_rp.ka_last >= g_rp.ka_period) {
        g_rp.ka_last = now;
        gb_repeat(g_rp.gb);
    }

    if (now - g_rp.last < g_rp.period)
        return;
    g_rp.last = now;

    unsigned char* disp = (unsigned char*)g_rp.fb;
    for (int i = 0; i < g_rp.nd; i++) {
        unsigned char r, g, b;
        class_color(g_rp.dets[i].class_id, &r, &g, &b);
        draw_rect(disp, g_rp.w, g_rp.h, g_rp.stride,
                  (int)g_rp.dets[i].left,  (int)g_rp.dets[i].top,
                  (int)g_rp.dets[i].right, (int)g_rp.dets[i].bottom,
                  r, g, b, 3);
    }

    if (g_rp.fire_on) {
        int x0 = (int)(g_rp.fire_x - g_rp.fire_w * 0.5f);
        int y0 = (int)(g_rp.fire_y - g_rp.fire_h * 0.5f);
        int x1 = (int)(g_rp.fire_x + g_rp.fire_w * 0.5f);
        int y1 = (int)(g_rp.fire_y + g_rp.fire_h * 0.5f);
        draw_rect(disp, g_rp.w, g_rp.h, g_rp.stride,
                  x0 - 6, y0 - 6, x1 + 6, y1 + 6, 255, 0, 255, 3);
        draw_rect(disp, g_rp.w, g_rp.h, g_rp.stride,
                  x0 - 12, y0 - 12, x1 + 12, y1 + 12, 255, 0, 255, 3);

        if (y0 - 12 < 0) y0 = 12;
        if (y1 + 12 >= g_rp.h) y1 = g_rp.h - 13;
        if (y1 >= y0)
            Xil_DCacheFlushRange(g_rp.fb +
                                 (unsigned int)(y0 - 12) * g_rp.stride,
                                 (unsigned int)(y1 - y0 + 25) * g_rp.stride);
    }

    draw_hud();

    for (int i = 0; i < g_rp.nd; i++) {
        int y0 = (int)g_rp.dets[i].top;
        int y1 = (int)g_rp.dets[i].bottom;
        if (y0 < 0) y0 = 0;
        if (y1 >= g_rp.h) y1 = g_rp.h - 1;
        if (y1 < y0) continue;
        Xil_DCacheFlushRange(g_rp.fb + (unsigned int)y0 * g_rp.stride,
                             (unsigned int)(y1 - y0 + 1) * g_rp.stride);
    }
    g_rp.count++;
}

void detect_loop(Yolo4_Tiny* yolo,
                 unsigned int fb_cap, unsigned int fb_disp,
                 int w, int h, int n_frames)
{
    const int stride = w * 3;
    const unsigned int fb_bytes = (unsigned int)stride * (unsigned int)h;

    printf("\r\n=== Detect loop %dx%d ===\r\n", w, h);
    printf("fb_cap=0x%08X  fb_disp=0x%08X\r\n", fb_cap, fb_disp);
    printf("frame  pre     inf    post   draw    total    dets  fps\r\n");

    unsigned long acc_pre = 0, acc_inf = 0, acc_post = 0, acc_draw = 0;
    int frame = 0;

    while (n_frames <= 0 || frame < n_frames) {
        XTime t0, t1, t2, t3, t4;

        XTime_GetTime(&t0);
        Xil_DCacheInvalidateRange(fb_cap, fb_bytes);
        yolo_preprocess((const unsigned char*)fb_cap, w, h, stride,
                        0, 0, w, h, YPRE_BILINEAR, g_net_in);

        Xil_DCacheFlushRange((unsigned int)(unsigned long)g_net_in,
                             (unsigned int)sizeof(g_net_in));

        XTime_GetTime(&t1);
        yolo->forward(g_net_in, g_out0, g_out1);

        Xil_DCacheInvalidateRange((unsigned int)(unsigned long)g_out0,
                                  (unsigned int)sizeof(g_out0));
        Xil_DCacheInvalidateRange((unsigned int)(unsigned long)g_out1,
                                  (unsigned int)sizeof(g_out1));

        XTime_GetTime(&t2);
        int nd = yolo_postprocess_q(g_out0, g_out1, w, h,
                                    0.5f, 0.3f, g_dets, YP_MAX_DET);

        XTime_GetTime(&t3);
        memcpy((void*)fb_disp, (const void*)fb_cap, (size_t)fb_bytes);

        unsigned char* disp = (unsigned char*)fb_disp;
        for (int i = 0; i < nd; i++) {
            unsigned char r, g, b;
            class_color(g_dets[i].class_id, &r, &g, &b);
            draw_rect(disp, w, h, stride,
                      (int)g_dets[i].left,  (int)g_dets[i].top,
                      (int)g_dets[i].right, (int)g_dets[i].bottom,
                      r, g, b, 2);
        }

        Xil_DCacheFlushRange(fb_disp, fb_bytes);
        XTime_GetTime(&t4);

        unsigned long us_pre  = (unsigned long)(((t1 - t0) * 1000000) / COUNTS_PER_SECOND);
        unsigned long us_inf  = (unsigned long)(((t2 - t1) * 1000000) / COUNTS_PER_SECOND);
        unsigned long us_post = (unsigned long)(((t3 - t2) * 1000000) / COUNTS_PER_SECOND);
        unsigned long us_draw = (unsigned long)(((t4 - t3) * 1000000) / COUNTS_PER_SECOND);
        unsigned long us_tot  = us_pre + us_inf + us_post + us_draw;

        acc_pre += us_pre; acc_inf += us_inf;
        acc_post += us_post; acc_draw += us_draw;
        frame++;

        printf("[%3d] %5lu %7lu %5lu %6lu  %7lu   %2d   %lu.%02lu\r\n",
               frame, us_pre, us_inf, us_post, us_draw, us_tot, nd,
               us_tot ? 1000000UL / us_tot : 0,
               us_tot ? (100000000UL / us_tot) % 100 : 0);

        for (int i = 0; i < nd && i < 5; i++) {
            int s100 = (int)(g_dets[i].score * 100.0f + 0.5f);
            printf("      %s %d.%02d  [%d %d %d %d]\r\n",
                   yp_class_names[g_dets[i].class_id], s100 / 100, s100 % 100,
                   (int)g_dets[i].left, (int)g_dets[i].top,
                   (int)g_dets[i].right, (int)g_dets[i].bottom);
        }
    }

    if (frame > 0) {
        unsigned long a_pre  = acc_pre / frame,  a_inf  = acc_inf / frame;
        unsigned long a_post = acc_post / frame, a_draw = acc_draw / frame;
        unsigned long a_tot  = a_pre + a_inf + a_post + a_draw;
        printf("\r\n--- average over %d frames ---\r\n", frame);
        printf("  pre   %6lu us  %3lu%%\r\n", a_pre,  a_tot ? a_pre  * 100 / a_tot : 0);
        printf("  inf   %6lu us  %3lu%%\r\n", a_inf,  a_tot ? a_inf  * 100 / a_tot : 0);
        printf("  post  %6lu us  %3lu%%\r\n", a_post, a_tot ? a_post * 100 / a_tot : 0);
        printf("  draw  %6lu us  %3lu%%\r\n", a_draw, a_tot ? a_draw * 100 / a_tot : 0);
        printf("  total %6lu us -> %lu.%02lu fps\r\n", a_tot,
               a_tot ? 1000000UL / a_tot : 0,
               a_tot ? (100000000UL / a_tot) % 100 : 0);
    }
}

void detect_loop_smooth(Yolo4_Tiny* yolo, unsigned int fb_cap,
                        int w, int h, int n_frames)
{
    const int stride = w * 3;
    const unsigned int fb_bytes = (unsigned int)stride * (unsigned int)h;

    printf("\r\n=== Detect loop (smooth) %dx%d ===\r\n", w, h);
    printf("display reads fb_cap directly -> smooth video\r\n");
    printf("frame  pre     inf    post   draw    total    dets  fps\r\n");

    unsigned long acc_tot = 0;
    int frame = 0;

    static YoloBox last_dets[YP_MAX_DET];
    int last_nd = 0;

    unsigned long last_sum = 0;
    int same_cnt = 0;

    g_rp.fb = fb_cap;
    g_rp.w = w; g_rp.h = h; g_rp.stride = stride;
    g_rp.dets = last_dets;
    g_rp.nd = 0;
    g_rp.period = COUNTS_PER_SECOND / 80;
    g_rp.gb = 0;
    g_rp.ka_period = COUNTS_PER_SECOND / 20;
    g_rp.count = 0;
    XTime_GetTime(&g_rp.last);
    g_conv_idle_hook = repaint_hook;

    while (n_frames <= 0 || frame < n_frames) {
        XTime t0, t1, t2, t3, t4;

        XTime_GetTime(&t0);
        Xil_DCacheInvalidateRange(fb_cap, fb_bytes);
        yolo_preprocess((const unsigned char*)fb_cap, w, h, stride,
                        0, 0, w, h, YPRE_BILINEAR, g_net_in);
        Xil_DCacheFlushRange((unsigned int)(unsigned long)g_net_in,
                             (unsigned int)sizeof(g_net_in));

        {
            unsigned long sum = 0;
            const int npx = YPRE_NET_SIZE * YPRE_NET_SIZE;
            for (int i = 0; i < 3 * npx; i += 997)
                sum += (unsigned long)(unsigned short)g_net_in[i];
            if (sum == last_sum)
                same_cnt++;
            else
                same_cnt = 0;
            last_sum = sum;
            if (same_cnt == 5)
                printf("  !! input identical for 5 frames -- camera not "
                       "updating? check VDMA EnableSync\r\n");
        }

        XTime_GetTime(&t1);
        yolo->forward(g_net_in, g_out0, g_out1);
        Xil_DCacheInvalidateRange((unsigned int)(unsigned long)g_out0,
                                  (unsigned int)sizeof(g_out0));
        Xil_DCacheInvalidateRange((unsigned int)(unsigned long)g_out1,
                                  (unsigned int)sizeof(g_out1));

        XTime_GetTime(&t2);
        int nd = yolo_postprocess_q(g_out0, g_out1, w, h,
                                    0.5f, 0.3f, g_dets, YP_MAX_DET);

        g_rp.nd = 0;
        for (int i = 0; i < nd; i++)
            last_dets[i] = g_dets[i];
        last_nd = nd;
        g_rp.nd = nd;

        XTime_GetTime(&t3);
        unsigned char* disp = (unsigned char*)fb_cap;
        for (int i = 0; i < last_nd; i++) {
            unsigned char r, g, b;
            class_color(last_dets[i].class_id, &r, &g, &b);
            draw_rect(disp, w, h, stride,
                      (int)last_dets[i].left,  (int)last_dets[i].top,
                      (int)last_dets[i].right, (int)last_dets[i].bottom,
                      r, g, b, 3);
        }
        Xil_DCacheFlushRange(fb_cap, fb_bytes);
        XTime_GetTime(&t4);

        unsigned long us_pre  = (unsigned long)(((t1 - t0) * 1000000) / COUNTS_PER_SECOND);
        unsigned long us_inf  = (unsigned long)(((t2 - t1) * 1000000) / COUNTS_PER_SECOND);
        unsigned long us_post = (unsigned long)(((t3 - t2) * 1000000) / COUNTS_PER_SECOND);
        unsigned long us_draw = (unsigned long)(((t4 - t3) * 1000000) / COUNTS_PER_SECOND);
        unsigned long us_tot  = us_pre + us_inf + us_post + us_draw;
        acc_tot += us_tot;
        frame++;

        printf("[%3d] %5lu %7lu %5lu %6lu  %7lu   %2d   %lu.%02lu  rp=%lu\r\n",
               frame, us_pre, us_inf, us_post, us_draw, us_tot, nd,
               us_tot ? 1000000UL / us_tot : 0,
               us_tot ? (100000000UL / us_tot) % 100 : 0,
               g_rp.count);
        g_rp.count = 0;
        for (int i = 0; i < nd && i < 5; i++) {
            int s100 = (int)(g_dets[i].score * 100.0f + 0.5f);
            printf("      %s %d.%02d  [%d %d %d %d]\r\n",
                   yp_class_names[g_dets[i].class_id], s100 / 100, s100 % 100,
                   (int)g_dets[i].left, (int)g_dets[i].top,
                   (int)g_dets[i].right, (int)g_dets[i].bottom);
        }
    }

    g_conv_idle_hook = 0;

    if (frame > 0) {
        unsigned long a = acc_tot / frame;
        printf("\r\n--- smooth: %d frames, avg %lu us -> %lu.%02lu fps ---\r\n",
               frame, a, a ? 1000000UL / a : 0, a ? (100000000UL / a) % 100 : 0);
    }
}

#define DUTY_WIN_SEC   3
#define DUTY_CAP       256

static struct {
    unsigned long long dur[DUTY_CAP];
    unsigned long long busy[DUTY_CAP];
    int head, tail;
    unsigned long long sum_dur, sum_busy;
} g_duty;

static void duty_reset(void)
{
    g_duty.head = g_duty.tail = 0;
    g_duty.sum_dur = g_duty.sum_busy = 0;
}

static unsigned long duty_push(unsigned long long dur, unsigned long long busy)
{
    const unsigned long long win =
        (unsigned long long)COUNTS_PER_SECOND * DUTY_WIN_SEC;

    g_duty.dur[g_duty.head]  = dur;
    g_duty.busy[g_duty.head] = busy;
    g_duty.sum_dur  += dur;
    g_duty.sum_busy += busy;
    g_duty.head = (g_duty.head + 1) % DUTY_CAP;

    if (g_duty.head == g_duty.tail) {
        g_duty.sum_dur  -= g_duty.dur[g_duty.tail];
        g_duty.sum_busy -= g_duty.busy[g_duty.tail];
        g_duty.tail = (g_duty.tail + 1) % DUTY_CAP;
    }

    while (g_duty.sum_dur > win && g_duty.tail != g_duty.head) {
        g_duty.sum_dur  -= g_duty.dur[g_duty.tail];
        g_duty.sum_busy -= g_duty.busy[g_duty.tail];
        g_duty.tail = (g_duty.tail + 1) % DUTY_CAP;
    }

    return g_duty.sum_dur
         ? (unsigned long)((g_duty.sum_busy * 100) / g_duty.sum_dur)
         : 0;
}

void detect_loop_event(Yolo4_Tiny* yolo, unsigned int fb_cap,
                       int w, int h, int n_frames,
                       int cell_thres, int cell_count)
{
    const int stride = w * 3;
    const unsigned int fb_bytes = (unsigned int)stride * (unsigned int)h;

    printf("\r\n=== Detect loop (event-driven) %dx%d ===\r\n", w, h);
    printf("motion grid %dx%d, cell_thres=%d, cell_count=%d\r\n",
           MO_GRID_W, MO_GRID_H, cell_thres, cell_count);
    printf("duty = accelerator busy time / wall time, over last %d s\r\n",
           DUTY_WIN_SEC);
    printf("tick  mo  cells  motion_us  infer_us   dets  duty%%\r\n");

    static YoloBox last_dets[YP_MAX_DET];
    int last_nd = 0;

    g_rp.fb = fb_cap;
    g_rp.w = w; g_rp.h = h; g_rp.stride = stride;
    g_rp.dets = last_dets;
    g_rp.nd = 0;
    g_rp.period = COUNTS_PER_SECOND / 80;
    g_rp.gb = 0;
    g_rp.ka_period = COUNTS_PER_SECOND / 20;
    g_rp.count = 0;
    XTime_GetTime(&g_rp.last);
    g_conv_idle_hook = repaint_hook;

    motion_reset();
    duty_reset();

    XTime t_start;
    XTime_GetTime(&t_start);
    unsigned long long busy_ticks = 0;
    int tick = 0, n_infer = 0;

    while (n_frames <= 0 || tick < n_frames) {
        XTime a, b, c;
        MotionResult mo;
        XTime tick_t0;
        XTime_GetTime(&tick_t0);

        XTime_GetTime(&a);
#if CLEAN_INPUT

        g_conv_idle_hook = 0;
        usleep(CAM_FRAME_US);
#endif
        Xil_DCacheInvalidateRange(fb_cap, fb_bytes);
        motion_detect((const unsigned char*)fb_cap, w, h, stride,
                      cell_thres, cell_count, &mo);
        XTime_GetTime(&b);

        unsigned long us_mo = (unsigned long)(((b - a) * 1000000) / COUNTS_PER_SECOND);
        unsigned long us_inf = 0;
        unsigned long long tick_busy = 0;
        int nd = last_nd;

        if (mo.moving) {
            yolo_preprocess((const unsigned char*)fb_cap, w, h, stride,
                            0, 0, w, h, YPRE_BILINEAR, g_net_in);
            Xil_DCacheFlushRange((unsigned int)(unsigned long)g_net_in,
                                 (unsigned int)sizeof(g_net_in));

#if CLEAN_INPUT
            g_conv_idle_hook = repaint_hook;
#endif
            yolo->forward(g_net_in, g_out0, g_out1);

            Xil_DCacheInvalidateRange((unsigned int)(unsigned long)g_out0,
                                      (unsigned int)sizeof(g_out0));
            Xil_DCacheInvalidateRange((unsigned int)(unsigned long)g_out1,
                                      (unsigned int)sizeof(g_out1));

            nd = yolo_postprocess_q(g_out0, g_out1, w, h,
                                    0.5f, 0.3f, g_dets, YP_MAX_DET);
            g_rp.nd = 0;
            for (int i = 0; i < nd; i++)
                last_dets[i] = g_dets[i];
            last_nd = nd;
            g_rp.nd = nd;
            n_infer++;

            XTime_GetTime(&c);
            tick_busy = (c - b);
            busy_ticks += tick_busy;
            us_inf = (unsigned long)((tick_busy * 1000000) / COUNTS_PER_SECOND);
        } else {
            unsigned char* disp = (unsigned char*)fb_cap;
            for (int i = 0; i < last_nd; i++) {
                unsigned char r, g, b2;
                class_color(last_dets[i].class_id, &r, &g, &b2);
                draw_rect(disp, w, h, stride,
                          (int)last_dets[i].left,  (int)last_dets[i].top,
                          (int)last_dets[i].right, (int)last_dets[i].bottom,
                          r, g, b2, 3);
            }
            Xil_DCacheFlushRange(fb_cap, fb_bytes);

            XTime w0;
            XTime_GetTime(&w0);
            while (1) {
                XTime wn;
                XTime_GetTime(&wn);
                if (wn - w0 >= (unsigned long long)(COUNTS_PER_SECOND / 25))
                    break;
            }
        }

        tick++;

        XTime tick_t1;
        XTime_GetTime(&tick_t1);
        unsigned long duty = duty_push(tick_t1 - tick_t0, tick_busy);

        printf("[%4d] %s %5d %8lu %9lu   %2d   %3lu\r\n",
               tick, mo.moving ? "MV" : "--", mo.changed_cells,
               us_mo, us_inf, nd, duty);
        if (mo.moving) {
            for (int i = 0; i < nd && i < 3; i++) {
                int s100 = (int)(last_dets[i].score * 100.0f + 0.5f);
                printf("       %s %d.%02d [%d %d %d %d]\r\n",
                       yp_class_names[last_dets[i].class_id],
                       s100 / 100, s100 % 100,
                       (int)last_dets[i].left,  (int)last_dets[i].top,
                       (int)last_dets[i].right, (int)last_dets[i].bottom);
            }
        }
    }

    g_conv_idle_hook = 0;

    XTime t_end;
    XTime_GetTime(&t_end);
    unsigned long long total = t_end - t_start;
    printf("\r\n--- event-driven summary ---\r\n");
    printf("  ticks           %d\r\n", tick);
    printf("  inferences      %d  (%d%% of ticks)\r\n",
           n_infer, tick ? n_infer * 100 / tick : 0);
    printf("  accel duty      %lu%%\r\n",
           total ? (unsigned long)((busy_ticks * 100) / total) : 0);
    printf("  wall time       %lu ms\r\n",
           (unsigned long)((total * 1000) / COUNTS_PER_SECOND));
}

#include "track.h"

void detect_loop_track(Yolo4_Tiny* yolo, unsigned int fb_cap,
                       int w, int h, int n_frames,
                       int cell_thres, int cell_count,
                       int want_class, float lead_sec)
{
    const int stride = w * 3;
    const unsigned int fb_bytes = (unsigned int)stride * (unsigned int)h;

    printf("\r\n=== Detect loop (track + gimbal) %dx%d ===\r\n", w, h);
    printf("lead=%d ms, class=%d, virt %dx%d (STM32 CENTER %d/%d)\r\n",
           (int)(lead_sec * 1000.0f), want_class,
           GB_VIRT_W, GB_VIRT_H, GB_VIRT_W / 2, GB_VIRT_H / 2);
    printf("lines starting with 'X' are gimbal frames, the rest is debug\r\n");
    printf("tick mo cells   inf_us  dets trk    cx    cy  duty%%\r\n");

    static YoloBox last_dets[YP_MAX_DET];
    int last_nd = 0;

    Tracker trk;

    trk_init(&trk, want_class, 4, 250.0f);

    Gimbal gb;
    gb_init(&gb, 0.7f);

    g_rp.fb = fb_cap;
    g_rp.w = w; g_rp.h = h; g_rp.stride = stride;
    g_rp.dets = last_dets;
    g_rp.nd = 0;
    g_rp.period = COUNTS_PER_SECOND / 80;
    g_rp.gb = &gb;
    g_rp.fire_on = 0;
    g_rp.ka_period = COUNTS_PER_SECOND / 20;
    g_rp.count = 0;

    g_rp.hud_on = 0;
    motion_set_ignore_bottom(0);
    XTime_GetTime(&g_rp.last);
    g_conv_idle_hook = repaint_hook;

    motion_reset();
    duty_reset();

    XTime t_start, t_prev;
    XTime_GetTime(&t_start);
    t_prev = t_start;
    unsigned long long busy_ticks = 0;
    int tick = 0, n_infer = 0;

    while (n_frames <= 0 || tick < n_frames) {
        XTime a, b, c, tick_t0;
        MotionResult mo;
        XTime_GetTime(&tick_t0);

        float dt = (float)((double)(tick_t0 - t_prev) / (double)COUNTS_PER_SECOND);
        t_prev = tick_t0;
        if (dt <= 0.0f) dt = 0.001f;

        XTime_GetTime(&a);
#if CLEAN_INPUT

        g_conv_idle_hook = 0;
        usleep(CAM_FRAME_US);
#endif
        Xil_DCacheInvalidateRange(fb_cap, fb_bytes);
        motion_detect((const unsigned char*)fb_cap, w, h, stride,
                      cell_thres, cell_count, &mo);
        XTime_GetTime(&b);

        unsigned long us_mo = (unsigned long)(((b - a) * 1000000) / COUNTS_PER_SECOND);
        unsigned long us_inf = 0;
        unsigned long long tick_busy = 0;
        int nd = last_nd;
        TrackOut to;

        if (mo.moving) {
            yolo_preprocess((const unsigned char*)fb_cap, w, h, stride,
                            0, 0, w, h, YPRE_BILINEAR, g_net_in);
            Xil_DCacheFlushRange((unsigned int)(unsigned long)g_net_in,
                                 (unsigned int)sizeof(g_net_in));

#if CLEAN_INPUT
            g_conv_idle_hook = repaint_hook;
#endif
            yolo->forward(g_net_in, g_out0, g_out1);

            Xil_DCacheInvalidateRange((unsigned int)(unsigned long)g_out0,
                                      (unsigned int)sizeof(g_out0));
            Xil_DCacheInvalidateRange((unsigned int)(unsigned long)g_out1,
                                      (unsigned int)sizeof(g_out1));

            nd = yolo_postprocess_q(g_out0, g_out1, w, h,
                                    0.5f, 0.3f, g_dets, YP_MAX_DET);
            g_rp.nd = 0;
            for (int i = 0; i < nd; i++)
                last_dets[i] = g_dets[i];
            last_nd = nd;
            g_rp.nd = nd;
            n_infer++;

            XTime_GetTime(&c);
            tick_busy = (c - b);
            busy_ticks += tick_busy;
            us_inf = (unsigned long)((tick_busy * 1000000) / COUNTS_PER_SECOND);

            trk_update(&trk, last_dets, last_nd, dt, lead_sec, &to);
        } else {
            trk_coast(&trk, dt, lead_sec, &to);

            unsigned char* disp = (unsigned char*)fb_cap;
            for (int i = 0; i < last_nd; i++) {
                unsigned char r, g, b2;
                class_color(last_dets[i].class_id, &r, &g, &b2);
                draw_rect(disp, w, h, stride,
                          (int)last_dets[i].left,  (int)last_dets[i].top,
                          (int)last_dets[i].right, (int)last_dets[i].bottom,
                          r, g, b2, 3);
            }
            Xil_DCacheFlushRange(fb_cap, fb_bytes);

            XTime w0;
            XTime_GetTime(&w0);
            while (1) {
                XTime wn;
                XTime_GetTime(&wn);
                if (wn - w0 >= (unsigned long long)(COUNTS_PER_SECOND / 25))
                    break;
            }
        }

        int nx = -1, ny = -1;
        if (to.valid) {
            unsigned char* disp = (unsigned char*)fb_cap;
            int kx0 = (int)(to.raw_cx - to.w * 0.5f);
            int ky0 = (int)(to.raw_cy - to.h * 0.5f);
            int kx1 = (int)(to.raw_cx + to.w * 0.5f);
            int ky1 = (int)(to.raw_cy + to.h * 0.5f);
            draw_rect(disp, w, h, stride, kx0, ky0, kx1, ky1, 255, 255, 0, 2);

            draw_rect(disp, w, h, stride,
                      (int)to.cx - 12, (int)to.cy - 1,
                      (int)to.cx + 12, (int)to.cy + 1, 255, 0, 0, 2);
            draw_rect(disp, w, h, stride,
                      (int)to.cx - 1, (int)to.cy - 12,
                      (int)to.cx + 1, (int)to.cy + 12, 255, 0, 0, 2);
            Xil_DCacheFlushRange(fb_cap, fb_bytes);

            gb_send(&gb, to.cx, to.cy, w, h, 0, &nx, &ny);
        } else {
            gb_lost(&gb);
        }

        tick++;
        XTime tick_t1;
        XTime_GetTime(&tick_t1);
        unsigned long duty = duty_push(tick_t1 - tick_t0, tick_busy);

        printf("[%4d] %s %5d %8lu  %2d  %s %5d %5d  %3lu\r\n",
               tick, mo.moving ? "MV" : "--", mo.changed_cells, us_inf, nd,
               to.valid ? (to.measured ? "M" : "P") : "-",
               nx, ny, duty);
    }

    g_conv_idle_hook = 0;

    XTime t_end;
    XTime_GetTime(&t_end);
    unsigned long long total = t_end - t_start;
    printf("\r\n--- track summary ---\r\n");
    printf("  ticks        %d\r\n", tick);
    printf("  inferences   %d (%d%%)\r\n", n_infer, tick ? n_infer * 100 / tick : 0);
    printf("  accel duty   %lu%%\r\n",
           total ? (unsigned long)((busy_ticks * 100) / total) : 0);
    printf("  gimbal sent  %lu, lost %lu\r\n", gb.sent, gb.skipped);
    printf("  track hits   %d, lost %d\r\n", trk.hits, trk.lost);
}

#include "laser.h"
#include "calib.h"
#include "bench_cpu.h"

void detect_loop_weed(Yolo4_Tiny* yolo, unsigned int fb_cap,
                      int w, int h, int n_frames,
                      int cell_thres, int cell_count,
                      float lead_sec,
                      const Calib* cal, const LaserZone* zone,
                      float aim_off_x, float aim_off_y)
{
    const int stride = w * 3;
    const unsigned int fb_bytes = (unsigned int)stride * (unsigned int)h;

    printf("\r\n=== Laser weeding loop %dx%d ===\r\n", w, h);
    printf("lead=%d ms, zone=(%d,%d)-(%d,%d), calib=%s\r\n",
           (int)(lead_sec * 1000.0f),
           (int)zone->x0, (int)zone->y0, (int)zone->x1, (int)zone->y1,
           (cal && cal->valid) ? "yes" : "NO (detect only)");
    printf("lines starting with 'X'/'A' are gimbal frames\r\n");
    printf("keys: q=stop+save  f=toggle ff  c=clear done  m=mark\r\n");
    printf("tick mo  inf_us weed crop trk  aim_px laser puls fired blk  why  duty%% move spd\r\n");

    g_log_n = 0;
    g_log_full = 0;

    static YoloBox last_dets[YP_MAX_DET];
    int last_nd = 0;

    Tracker trk;

    trk_init(&trk, YP_CLS_WEED, 4, 120.0f);

    Gimbal gb;
    gb_init(&gb, 0.7f);

    Laser las;
    ls_init(&las, zone);

    g_rp.fb = fb_cap;
    g_rp.w = w; g_rp.h = h; g_rp.stride = stride;
    g_rp.dets = last_dets;
    g_rp.nd = 0;
    g_rp.period = COUNTS_PER_SECOND / 80;
    g_rp.gb = &gb;
    g_rp.fire_on = 0;
    g_rp.ka_period = COUNTS_PER_SECOND / 20;
    g_rp.count = 0;

    g_rp.hud_on = 0;
    motion_set_ignore_bottom(0);

    XTime_GetTime(&g_rp.last);
    g_conv_idle_hook = repaint_hook;

    motion_reset();
    duty_reset();

    XTime t_start, t_prev;
    XTime_GetTime(&t_start);
    t_prev = t_start;
    unsigned long long busy_ticks = 0;
    int tick = 0, n_infer = 0;

    long prev_gx = 0, prev_gy = 0;
    long gmove = 0;

    int clear_cnt = 0;
    int skip_run  = 0;

    int ff_on = (lead_sec > 0.001f);
    int run_mark = 0;

    int static_mode = 0;

    float aim_x = -1.0f, aim_y = -1.0f;

    float trk_score = 0.0f;

    while (n_frames <= 0 || tick < n_frames) {
        XTime a, b, c, tick_t0;
        MotionResult mo;
        XTime_GetTime(&tick_t0);

        float dt = (float)((double)(tick_t0 - t_prev) / (double)COUNTS_PER_SECOND);
        t_prev = tick_t0;
        if (dt <= 0.0f) dt = 0.001f;

        unsigned long now_ms =
            (unsigned long)((double)tick_t0 * 1000.0 / (double)COUNTS_PER_SECOND);
        trk_set_time(&trk, now_ms);

        XTime_GetTime(&a);
#if CLEAN_INPUT

        g_conv_idle_hook = 0;
        usleep(CAM_FRAME_US);
#endif
        Xil_DCacheInvalidateRange(fb_cap, fb_bytes);
        motion_detect((const unsigned char*)fb_cap, w, h, stride,
                      cell_thres, cell_count, &mo);
        XTime_GetTime(&b);

        unsigned long us_inf = 0;
        unsigned long long tick_busy = 0;
        int nd = last_nd;
        int n_weed = 0, n_crop = 0;
        TrackOut to;

        int weed_pending = 0;
        for (int i = 0; i < last_nd; i++) {
            if (last_dets[i].class_id != YP_CLS_WEED)
                continue;
            float cx = (last_dets[i].left + last_dets[i].right) * 0.5f;
            float cy = (last_dets[i].top + last_dets[i].bottom) * 0.5f;
            if (!ls_is_done(&las, cx, cy)) {
                weed_pending = 1;
                break;
            }
        }

        if (weed_pending || las.state != LS_IDLE)
            clear_cnt = 0;
        else if (clear_cnt < CLEAR_CONFIRM)
            clear_cnt++;

        int probe = (skip_run >= IDLE_PROBE);

        const int did_infer = (mo.moving || clear_cnt < CLEAR_CONFIRM || probe);

        if (did_infer) {
            skip_run = 0;
        } else {
            skip_run++;
        }

        if (did_infer) {
            yolo_preprocess((const unsigned char*)fb_cap, w, h, stride,
                            0, 0, w, h, YPRE_BILINEAR, g_net_in);
            Xil_DCacheFlushRange((unsigned int)(unsigned long)g_net_in,
                                 (unsigned int)sizeof(g_net_in));

#if CLEAN_INPUT
            g_conv_idle_hook = repaint_hook;
#endif
            yolo->forward(g_net_in, g_out0, g_out1);

            Xil_DCacheInvalidateRange((unsigned int)(unsigned long)g_out0,
                                      (unsigned int)sizeof(g_out0));
            Xil_DCacheInvalidateRange((unsigned int)(unsigned long)g_out1,
                                      (unsigned int)sizeof(g_out1));

            nd = yolo_postprocess_q(g_out0, g_out1, w, h,
                                    0.5f, 0.3f, g_dets, YP_MAX_DET);
            g_rp.nd = 0;
            for (int i = 0; i < nd; i++) {
                last_dets[i] = g_dets[i];
                if (g_dets[i].class_id == YP_CLS_WEED) n_weed++;
                else if (g_dets[i].class_id == YP_CLS_CROP) n_crop++;
            }
            last_nd = nd;
            g_rp.nd = nd;
            n_infer++;

            XTime_GetTime(&c);
            tick_busy = (c - b);
            busy_ticks += tick_busy;
            us_inf = (unsigned long)((tick_busy * 1000000) / COUNTS_PER_SECOND);

            trk_update(&trk, last_dets, last_nd, dt,
                       ff_on ? lead_sec : 0.0f, &to);

            if (to.valid && to.measured) {
                trk_score = 0.0f;
                float best = 1e18f;
                for (int i = 0; i < last_nd; i++) {
                    if (last_dets[i].class_id != YP_CLS_WEED) continue;
                    float cx = (last_dets[i].left + last_dets[i].right) * 0.5f;
                    float cy = (last_dets[i].top + last_dets[i].bottom) * 0.5f;
                    float dx = cx - to.raw_cx, dy = cy - to.raw_cy;
                    float d2 = dx * dx + dy * dy;
                    if (d2 < best) { best = d2; trk_score = last_dets[i].score; }
                }
            }
        } else {
            trk_coast(&trk, dt, ff_on ? lead_sec : 0.0f, &to);

            unsigned char* disp = (unsigned char*)fb_cap;
            for (int i = 0; i < last_nd; i++) {
                unsigned char r, g, b2;
                class_color(last_dets[i].class_id, &r, &g, &b2);
                draw_rect(disp, w, h, stride,
                          (int)last_dets[i].left,  (int)last_dets[i].top,
                          (int)last_dets[i].right, (int)last_dets[i].bottom,
                          r, g, b2, 3);
            }
            Xil_DCacheFlushRange(fb_cap, fb_bytes);

            XTime w0;
            XTime_GetTime(&w0);
            while (1) {
                XTime wn;
                XTime_GetTime(&wn);
                if (wn - w0 >= (unsigned long long)(COUNTS_PER_SECOND / 25))
                    break;
            }
        }

        float spd_mm_s = 0.0f;
        if (to.valid)
            spd_mm_s = sqrtf(to.vx * to.vx + to.vy * to.vy) * 0.5375f;

        float aim_err = 1e9f;
        if (to.valid) {
            if (cal && cal->valid && gb.has_last && gb.last_is_pos) {
                float tgx = 0.0f, tgy = 0.0f;
                cal_map(cal, to.raw_cx, to.raw_cy, &tgx, &tgy);

                float sx = fabsf(cal->a[0]);
                float sy = fabsf(cal->b[1]);
                if (sx < 1e-6f) sx = 1.0f;
                if (sy < 1e-6f) sy = 1.0f;
                float ex = (tgx - (float)gb.last_nx) / sx;
                float ey = (tgy - (float)gb.last_ny) / sy;
                aim_err = sqrtf(ex * ex + ey * ey);
            } else if (aim_x >= 0.0f) {
                float dx = to.cx - aim_x;
                float dy = to.cy - aim_y;
                aim_err = sqrtf(dx * dx + dy * dy);
            }
        }

        int laser_on = ls_tick(&las, to.valid, to.measured, to.class_id,
                               trk_score, to.cx, to.cy, aim_err, trk.uid,
                               to.vx, to.vy, now_ms);

        if (las.just_done) {
            trk_mark_done(&trk, to.raw_cx, to.raw_cy, to.vx, to.vy,
                          las.done_radius_px);
            trk_release(&trk);
        }

        int nx = -1, ny = -1;
        if (to.valid) {
            unsigned char* disp = (unsigned char*)fb_cap;

            draw_rect(disp, w, h, stride,
                      (int)(to.raw_cx - to.w * 0.5f), (int)(to.raw_cy - to.h * 0.5f),
                      (int)(to.raw_cx + to.w * 0.5f), (int)(to.raw_cy + to.h * 0.5f),
                      255, 255, 0, 2);

            draw_rect(disp, w, h, stride,
                      (int)to.cx - 12, (int)to.cy - 1,
                      (int)to.cx + 12, (int)to.cy + 1, 255, 0, 0, 2);
            draw_rect(disp, w, h, stride,
                      (int)to.cx - 1, (int)to.cy - 12,
                      (int)to.cx + 1, (int)to.cy + 12, 255, 0, 0, 2);

            g_rp.fire_on = laser_on;
            g_rp.fire_x  = to.cx;
            g_rp.fire_y  = to.cy;
            g_rp.fire_w  = to.w;
            g_rp.fire_h  = to.h;
            Xil_DCacheFlushRange(fb_cap, fb_bytes);

            if (cal && cal->valid) {
                float fgx = 0.0f, fgy = 0.0f;

                cal_map(cal, to.cx + aim_off_x, to.cy + aim_off_y, &fgx, &fgy);
                long cgx = (long)(fgx + (fgx >= 0 ? 0.5f : -0.5f));
                long cgy = (long)(fgy + (fgy >= 0 ? 0.5f : -0.5f));

                long d1 = cgx - prev_gx, d2 = cgy - prev_gy;
                if (d1 < 0) d1 = -d1;
                if (d2 < 0) d2 = -d2;
                gmove = (d1 > d2) ? d1 : d2;
                prev_gx = cgx;
                prev_gy = cgy;
                gp_send(&gb, cgx, cgy, laser_on, &nx, &ny);
            } else {
                gb_send(&gb, to.cx, to.cy, w, h, laser_on, &nx, &ny);
            }
            aim_x = to.cx;
            aim_y = to.cy;
        } else {
            gb_lost(&gb);
            ls_abort(&las);
            aim_x = aim_y = -1.0f;
            trk_score = 0.0f;
            g_rp.fire_on = 0;
        }

        tick++;
        XTime tick_t1;
        XTime_GetTime(&tick_t1);
        unsigned long duty = duty_push(tick_t1 - tick_t0, tick_busy);

        {
            float dt_s = (float)((double)(tick_t1 - tick_t0)
                                 / (double)COUNTS_PER_SECOND);
            if (dt_s < 0.0f)   dt_s = 0.0f;
            if (dt_s > 0.5f)   dt_s = 0.5f;

            int target = (int)duty;
            float cur  = (float)g_rp.duty;

            float tau  = (target > (int)cur) ? 0.8f : 2.0f;
            float k    = dt_s / tau;
            if (k > 1.0f) k = 1.0f;

            cur += ((float)target - cur) * k;

            int v = (int)(cur + 0.5f);

            if (v == g_rp.duty && v != target)
                v += (target > v) ? 1 : -1;
            g_rp.duty = v;
        }

        logf_both("[%4d] %s %7lu  %2d  %2d  %s  %5d  %s %4lu %5lu %4lu %s %3lu %5ld %4d\r\n",
                  tick, mo.moving ? "MV" : ((clear_cnt < CLEAR_CONFIRM || probe) ? "WK" : "--"),
                  us_inf, n_weed, n_crop,
                  to.valid ? (to.measured ? "M" : "P") : "-",
                  (aim_err > 9999.0f) ? 9999 : (int)aim_err,
                  ls_state_name(las.state),
                  las.n_pulse, las.n_fired, las.n_blocked,
                  ls_block_name(las.blk_why),
                  duty, gmove, (int)spd_mm_s);
        gmove = 0;

        if (XUartPs_IsReceiveData(XPAR_PS7_UART_0_BASEADDR)) {
            int kc = (int)XUartPs_RecvByte(XPAR_PS7_UART_0_BASEADDR);
            if (kc == 'q' || kc == 'Q')
                break;

            if (kc == 'f' || kc == 'F') {
                ff_on = !ff_on;
                logf_both("### feedforward %s (lead=%d ms)\r\n",
                          ff_on ? "ON" : "OFF",
                          ff_on ? (int)(lead_sec * 1000.0f) : 0);
            }

            if (kc == 'c' || kc == 'C') {
                ls_clear_done(&las);
                trk_clear_done(&trk);
                trk_release(&trk);
                logf_both("### done-list cleared\r\n");
            }

            if (kc == 'm' || kc == 'M') {
                run_mark++;
                logf_both("### mark %d\r\n", run_mark);
            }

            if (kc == 's' || kc == 'S') {
                static_mode = !static_mode;
                ff_on = !static_mode;
                las.fire_ticks = static_mode ? 6 : 1;

                ls_clear_done(&las);
                trk_clear_done(&trk);
                trk_release(&trk);
                ls_abort(&las);
                logf_both("### %s mode: lead=%d ms, fire_ticks=%d\r\n",
                          static_mode ? "STATIC" : "DYNAMIC",
                          ff_on ? (int)(lead_sec * 1000.0f) : 0,
                          las.fire_ticks);
            }
        }
    }

    g_conv_idle_hook = 0;
    ls_abort(&las);

    XTime t_end;
    XTime_GetTime(&t_end);
    unsigned long long total = t_end - t_start;

    logf_both("\r\n--- weeding summary ---\r\n");
    logf_both("  ticks         %d\r\n", tick);
    logf_both("  inferences    %d (%d%%)\r\n",
              n_infer, tick ? n_infer * 100 / tick : 0);
    logf_both("  accel duty    %lu%%\r\n",
              total ? (unsigned long)((busy_ticks * 100) / total) : 0);

    logf_both("  laser pulses  %lu\r\n", las.n_pulse);
    logf_both("  laser fired   %lu\r\n", las.n_fired);
    logf_both("  laser blocked %lu\r\n", las.n_blocked);
    logf_both("  gimbal sent   %lu, repeat %lu, lost %lu\r\n",
              gb.sent, gb.repeated, gb.skipped);
    logf_both("  zone          (%d,%d)-(%d,%d)\r\n",
              (int)zone->x0, (int)zone->y0, (int)zone->x1, (int)zone->y1);
    logf_both("  calib         %s\r\n",
              (cal && cal->valid) ? "yes (absolute position mode)" : "no");

    log_flush("run.txt");
}

void detect_loop_demo(Yolo4_Tiny* yolo, unsigned int fb_cap,
                      int w, int h, int n_frames,
                      int cell_thres, int cell_count,
                      float lead_sec,
                      const Calib* cal, const LaserZone* zone,
                      float aim_off_x, float aim_off_y)
{
    const int stride = w * 3;
    const unsigned int fb_bytes = (unsigned int)stride * (unsigned int)h;

    printf("\r\n=== DEMO (duty / bench / crop-protect) %dx%d ===\r\n", w, h);
    printf("lead=%d ms, zone=(%d,%d)-(%d,%d), calib=%s\r\n",
           (int)(lead_sec * 1000.0f),
           (int)zone->x0, (int)zone->y0, (int)zone->x1, (int)zone->y1,
           (cal && cal->valid) ? "yes" : "NO (detect only)");
    printf("lines starting with 'X'/'A' are gimbal frames\r\n");
    printf("keys: q=stop  v=duty demo  b=PL-vs-ARM  d=featmap  h=HUD  c=clear\r\n");
    printf("tick mo  inf_us weed crop trk  aim_px laser puls fired blk  why  duty%% move spd\r\n");

    g_log_n = 0;
    g_log_full = 0;

    static YoloBox last_dets[YP_MAX_DET];
    int last_nd = 0;

    Tracker trk;

    trk_init(&trk, YP_CLS_WEED, 4, 120.0f);

    Gimbal gb;
    gb_init(&gb, 0.7f);

    Laser las;
    ls_init(&las, zone);

    g_rp.fb = fb_cap;
    g_rp.w = w; g_rp.h = h; g_rp.stride = stride;
    g_rp.dets = last_dets;
    g_rp.nd = 0;
    g_rp.period = COUNTS_PER_SECOND / 80;
    g_rp.gb = &gb;
    g_rp.fire_on = 0;
    g_rp.ka_period = COUNTS_PER_SECOND / 20;
    g_rp.count = 0;

    g_rp.hud_h  = h / 12;
    if (g_rp.hud_h < 28) g_rp.hud_h = 28;
    g_rp.hud_on = 1;
    g_rp.duty = 0;
    motion_set_ignore_bottom(g_rp.hud_h);

    XTime_GetTime(&g_rp.last);
    g_conv_idle_hook = repaint_hook;

    motion_reset();
    duty_reset();

    XTime t_start, t_prev;
    XTime_GetTime(&t_start);
    t_prev = t_start;
    unsigned long long busy_ticks = 0;
    int tick = 0, n_infer = 0;

    long prev_gx = 0, prev_gy = 0;
    long gmove = 0;

    int clear_cnt = 0;
    int skip_run  = 0;

    int ff_on = (lead_sec > 0.001f);
    int run_mark = 0;

    int static_mode = 0;

    int demo_mode = 1;

    float aim_x = -1.0f, aim_y = -1.0f;

    float trk_score = 0.0f;

    while (n_frames <= 0 || tick < n_frames) {
        XTime a, b, c, tick_t0;
        MotionResult mo;
        XTime_GetTime(&tick_t0);

        float dt = (float)((double)(tick_t0 - t_prev) / (double)COUNTS_PER_SECOND);
        t_prev = tick_t0;
        if (dt <= 0.0f) dt = 0.001f;

        unsigned long now_ms =
            (unsigned long)((double)tick_t0 * 1000.0 / (double)COUNTS_PER_SECOND);
        trk_set_time(&trk, now_ms);

        XTime_GetTime(&a);
#if CLEAN_INPUT

        g_conv_idle_hook = 0;

        {
            XTime cw0;
            XTime_GetTime(&cw0);
            const unsigned long long cwdur =
                (unsigned long long)COUNTS_PER_SECOND * CAM_FRAME_US / 1000000ULL;
            while (1) {
                XTime cwn;
                XTime_GetTime(&cwn);
                if (cwn - cw0 >= cwdur)
                    break;
                draw_hud();
            }
        }
#endif
        Xil_DCacheInvalidateRange(fb_cap, fb_bytes);
        motion_detect((const unsigned char*)fb_cap, w, h, stride,
                      cell_thres, cell_count, &mo);
        XTime_GetTime(&b);

        draw_hud();

        unsigned long us_inf = 0;
        unsigned long long tick_busy = 0;
        int nd = last_nd;
        int n_weed = 0, n_crop = 0;
        TrackOut to;

        int weed_pending = 0;
        for (int i = 0; i < last_nd; i++) {
            if (last_dets[i].class_id != YP_CLS_WEED)
                continue;
            float cx = (last_dets[i].left + last_dets[i].right) * 0.5f;
            float cy = (last_dets[i].top + last_dets[i].bottom) * 0.5f;
            if (!ls_is_done(&las, cx, cy)) {
                weed_pending = 1;
                break;
            }
        }

        if (demo_mode) {
            if (mo.moving)
                clear_cnt = 0;
            else if (clear_cnt < CLEAR_CONFIRM)
                clear_cnt++;
        } else if (weed_pending || las.state != LS_IDLE) {
            clear_cnt = 0;
        } else if (clear_cnt < CLEAR_CONFIRM) {
            clear_cnt++;
        }

        int probe = (skip_run >= (demo_mode ? 600 : IDLE_PROBE));

        const int did_infer = (mo.moving || clear_cnt < CLEAR_CONFIRM || probe);

        char why_inf[4];
        why_inf[0] = mo.moving ? 'M' : '.';
        why_inf[1] = (clear_cnt < CLEAR_CONFIRM) ? 'C' : '.';
        why_inf[2] = probe ? 'P' : '.';
        why_inf[3] = '\0';

        if (did_infer) {
            skip_run = 0;
        } else {
            skip_run++;
        }

        if (did_infer) {
            yolo_preprocess((const unsigned char*)fb_cap, w, h, stride,
                            0, 0, w, h, YPRE_BILINEAR, g_net_in);
            Xil_DCacheFlushRange((unsigned int)(unsigned long)g_net_in,
                                 (unsigned int)sizeof(g_net_in));

#if CLEAN_INPUT
            g_conv_idle_hook = repaint_hook;
#endif
            yolo->forward(g_net_in, g_out0, g_out1);

            Xil_DCacheInvalidateRange((unsigned int)(unsigned long)g_out0,
                                      (unsigned int)sizeof(g_out0));
            Xil_DCacheInvalidateRange((unsigned int)(unsigned long)g_out1,
                                      (unsigned int)sizeof(g_out1));

            nd = yolo_postprocess_q(g_out0, g_out1, w, h,
                                    0.5f, 0.3f, g_dets, YP_MAX_DET);
            g_rp.nd = 0;
            for (int i = 0; i < nd; i++) {
                last_dets[i] = g_dets[i];
                if (g_dets[i].class_id == YP_CLS_WEED) n_weed++;
                else if (g_dets[i].class_id == YP_CLS_CROP) n_crop++;
            }
            last_nd = nd;
            g_rp.nd = nd;
            n_infer++;

            XTime_GetTime(&c);
            tick_busy = (c - b);
            busy_ticks += tick_busy;
            us_inf = (unsigned long)((tick_busy * 1000000) / COUNTS_PER_SECOND);

            trk_update(&trk, last_dets, last_nd, dt,
                       ff_on ? lead_sec : 0.0f, &to);

            if (to.valid && to.measured) {
                trk_score = 0.0f;
                float best = 1e18f;
                for (int i = 0; i < last_nd; i++) {
                    if (last_dets[i].class_id != YP_CLS_WEED) continue;
                    float cx = (last_dets[i].left + last_dets[i].right) * 0.5f;
                    float cy = (last_dets[i].top + last_dets[i].bottom) * 0.5f;
                    float dx = cx - to.raw_cx, dy = cy - to.raw_cy;
                    float d2 = dx * dx + dy * dy;
                    if (d2 < best) { best = d2; trk_score = last_dets[i].score; }
                }
            }
        } else {
            trk_coast(&trk, dt, ff_on ? lead_sec : 0.0f, &to);

            unsigned char* disp = (unsigned char*)fb_cap;
            for (int i = 0; i < last_nd; i++) {
                unsigned char r, g, b2;
                class_color(last_dets[i].class_id, &r, &g, &b2);
                draw_rect(disp, w, h, stride,
                          (int)last_dets[i].left,  (int)last_dets[i].top,
                          (int)last_dets[i].right, (int)last_dets[i].bottom,
                          r, g, b2, 3);
            }
            Xil_DCacheFlushRange(fb_cap, fb_bytes);

            XTime w0;
            XTime_GetTime(&w0);
            while (1) {
                XTime wn;
                XTime_GetTime(&wn);
                if (wn - w0 >= (unsigned long long)(COUNTS_PER_SECOND / 25))
                    break;
                draw_hud();
            }
        }

        float spd_mm_s = 0.0f;
        if (to.valid)
            spd_mm_s = sqrtf(to.vx * to.vx + to.vy * to.vy) * 0.5375f;

        float aim_err = 1e9f;
        if (to.valid) {
            if (cal && cal->valid && gb.has_last && gb.last_is_pos) {
                float tgx = 0.0f, tgy = 0.0f;
                cal_map(cal, to.raw_cx, to.raw_cy, &tgx, &tgy);

                float sx = fabsf(cal->a[0]);
                float sy = fabsf(cal->b[1]);
                if (sx < 1e-6f) sx = 1.0f;
                if (sy < 1e-6f) sy = 1.0f;
                float ex = (tgx - (float)gb.last_nx) / sx;
                float ey = (tgy - (float)gb.last_ny) / sy;
                aim_err = sqrtf(ex * ex + ey * ey);
            } else if (aim_x >= 0.0f) {
                float dx = to.cx - aim_x;
                float dy = to.cy - aim_y;
                aim_err = sqrtf(dx * dx + dy * dy);
            }
        }

        int laser_on = ls_tick(&las, demo_mode ? 0 : to.valid,
                               to.measured, to.class_id,
                               trk_score, to.cx, to.cy, aim_err, trk.uid,
                               to.vx, to.vy, now_ms);

        if (las.just_done) {
            trk_mark_done(&trk, to.raw_cx, to.raw_cy, to.vx, to.vy,
                          las.done_radius_px);
            trk_release(&trk);
        }

        int nx = -1, ny = -1;
        if (to.valid) {
            unsigned char* disp = (unsigned char*)fb_cap;

            draw_rect(disp, w, h, stride,
                      (int)(to.raw_cx - to.w * 0.5f), (int)(to.raw_cy - to.h * 0.5f),
                      (int)(to.raw_cx + to.w * 0.5f), (int)(to.raw_cy + to.h * 0.5f),
                      255, 255, 0, 2);

            draw_rect(disp, w, h, stride,
                      (int)to.cx - 12, (int)to.cy - 1,
                      (int)to.cx + 12, (int)to.cy + 1, 255, 0, 0, 2);
            draw_rect(disp, w, h, stride,
                      (int)to.cx - 1, (int)to.cy - 12,
                      (int)to.cx + 1, (int)to.cy + 12, 255, 0, 0, 2);

            g_rp.fire_on = laser_on;
            g_rp.fire_x  = to.cx;
            g_rp.fire_y  = to.cy;
            g_rp.fire_w  = to.w;
            g_rp.fire_h  = to.h;
            Xil_DCacheFlushRange(fb_cap, fb_bytes);

            if (cal && cal->valid) {
                float fgx = 0.0f, fgy = 0.0f;

                cal_map(cal, to.cx + aim_off_x, to.cy + aim_off_y, &fgx, &fgy);
                long cgx = (long)(fgx + (fgx >= 0 ? 0.5f : -0.5f));
                long cgy = (long)(fgy + (fgy >= 0 ? 0.5f : -0.5f));

                long d1 = cgx - prev_gx, d2 = cgy - prev_gy;
                if (d1 < 0) d1 = -d1;
                if (d2 < 0) d2 = -d2;
                gmove = (d1 > d2) ? d1 : d2;
                prev_gx = cgx;
                prev_gy = cgy;
                gp_send(&gb, cgx, cgy, laser_on, &nx, &ny);
            } else {
                gb_send(&gb, to.cx, to.cy, w, h, laser_on, &nx, &ny);
            }
            aim_x = to.cx;
            aim_y = to.cy;
        } else {
            gb_lost(&gb);
            ls_abort(&las);
            aim_x = aim_y = -1.0f;
            trk_score = 0.0f;
            g_rp.fire_on = 0;
        }

        tick++;
        XTime tick_t1;
        XTime_GetTime(&tick_t1);
        unsigned long duty = duty_push(tick_t1 - tick_t0, tick_busy);

        {
            float dt_s = (float)((double)(tick_t1 - tick_t0)
                                 / (double)COUNTS_PER_SECOND);
            if (dt_s < 0.0f)   dt_s = 0.0f;
            if (dt_s > 0.5f)   dt_s = 0.5f;

            int target = (int)duty;
            float cur  = (float)g_rp.duty;

            float tau  = (target > (int)cur) ? 0.8f : 2.0f;
            float k    = dt_s / tau;
            if (k > 1.0f) k = 1.0f;

            cur += ((float)target - cur) * k;

            int v = (int)(cur + 0.5f);

            if (v == g_rp.duty && v != target)
                v += (target > v) ? 1 : -1;
            g_rp.duty = v;
        }

        logf_both("[%4d] %s%4d %s%s %7lu  %2d  %2d  %s  %5d  %s %4lu %5lu %4lu %s %3lu %5ld %4d\r\n",
                  tick, mo.moving ? "MV" : ((clear_cnt < CLEAR_CONFIRM || probe) ? "WK" : "--"),
                  mo.changed_cells,
                  why_inf, demo_mode ? "*" : " ",
                  us_inf, n_weed, n_crop,
                  to.valid ? (to.measured ? "M" : "P") : "-",
                  (aim_err > 9999.0f) ? 9999 : (int)aim_err,
                  ls_state_name(las.state),
                  las.n_pulse, las.n_fired, las.n_blocked,
                  ls_block_name(las.blk_why),
                  duty, gmove, (int)spd_mm_s);
        gmove = 0;

        if (XUartPs_IsReceiveData(XPAR_PS7_UART_0_BASEADDR)) {
            int kc = (int)XUartPs_RecvByte(XPAR_PS7_UART_0_BASEADDR);
            if (kc == 'q' || kc == 'Q')
                break;

            if (kc == 'f' || kc == 'F') {
                ff_on = !ff_on;
                logf_both("### feedforward %s (lead=%d ms)\r\n",
                          ff_on ? "ON" : "OFF",
                          ff_on ? (int)(lead_sec * 1000.0f) : 0);
            }

            if (kc == 'c' || kc == 'C') {
                ls_clear_done(&las);
                trk_clear_done(&trk);
                trk_release(&trk);
                logf_both("### done-list cleared\r\n");
            }

            if (kc == 'm' || kc == 'M') {
                run_mark++;
                logf_both("### mark %d\r\n", run_mark);
            }

            if (kc == 'h' || kc == 'H') {
                g_rp.hud_on = !g_rp.hud_on;
                motion_set_ignore_bottom(g_rp.hud_on ? g_rp.hud_h : 0);
                logf_both("### HUD %s\r\n", g_rp.hud_on ? "ON" : "OFF");
            }

            if (kc == 'v' || kc == 'V') {
                demo_mode = !demo_mode;
                if (demo_mode) {
                    ls_abort(&las);
                    gp_send(&gb, gb.last_nx, gb.last_ny, 0, 0, 0);
                }
                logf_both("### DUTY DEMO %s\r\n", demo_mode ? "ON" : "OFF");
            }

            if (kc == 'd' || kc == 'D') {
                fd_request("feat");
                write_data("feat\\input_3x416x416.bin",
                           g_net_in, 3 * 416 * 416);
            }

            if (kc == 'b' || kc == 'B') {
                ls_abort(&las);
                gp_send(&gb, gb.last_nx, gb.last_ny, 0, 0, 0);

                g_conv_idle_hook = 0;

                g_bd.fb     = fb_cap;
                g_bd.w      = w;
                g_bd.h      = h;
                g_bd.stride = stride;
                g_bd.bar_w  = (w * 8) / 10;
                g_bd.bar_h  = 40;
                g_bd.x0     = (w - g_bd.bar_w) / 2;

                g_bd.y0     = h - 2 * (g_bd.bar_h + 14) - 24;
                g_bench_ms_pl = 0.0;
                g_bd.frac[0] = g_bd.frac[1] = 0.0f;
                g_bd.done[0] = g_bd.done[1] = 0;
                g_bd.active  = 1;

                {
                    XTime p0, pn;
                    XTime_GetTime(&p0);
                    do {
                        bench_repaint();
                        XTime_GetTime(&pn);
                    } while (pn - p0 < (unsigned long long)COUNTS_PER_SECOND);
                }

                bench_run(yolo, g_net_in, bench_draw, bench_repaint);

                {
                    XTime d0, dn;
                    XTime_GetTime(&d0);
                    do {
                        bench_repaint();
                        XTime_GetTime(&dn);
                    } while (dn - d0 <
                             (unsigned long long)COUNTS_PER_SECOND * 6);
                }

                g_bd.active = 0;
                g_conv_idle_hook = repaint_hook;
            }

            if (kc == 's' || kc == 'S') {
                static_mode = !static_mode;
                ff_on = !static_mode;
                las.fire_ticks = static_mode ? 6 : 1;

                ls_clear_done(&las);
                trk_clear_done(&trk);
                trk_release(&trk);
                ls_abort(&las);
                logf_both("### %s mode: lead=%d ms, fire_ticks=%d\r\n",
                          static_mode ? "STATIC" : "DYNAMIC",
                          ff_on ? (int)(lead_sec * 1000.0f) : 0,
                          las.fire_ticks);
            }
        }
    }

    g_conv_idle_hook = 0;
    ls_abort(&las);

    XTime t_end;
    XTime_GetTime(&t_end);
    unsigned long long total = t_end - t_start;

    logf_both("\r\n--- weeding summary ---\r\n");
    logf_both("  ticks         %d\r\n", tick);
    logf_both("  inferences    %d (%d%%)\r\n",
              n_infer, tick ? n_infer * 100 / tick : 0);
    logf_both("  accel duty    %lu%%\r\n",
              total ? (unsigned long)((busy_ticks * 100) / total) : 0);

    logf_both("  laser pulses  %lu\r\n", las.n_pulse);
    logf_both("  laser fired   %lu\r\n", las.n_fired);
    logf_both("  laser blocked %lu\r\n", las.n_blocked);
    logf_both("  gimbal sent   %lu, repeat %lu, lost %lu\r\n",
              gb.sent, gb.repeated, gb.skipped);
    logf_both("  zone          (%d,%d)-(%d,%d)\r\n",
              (int)zone->x0, (int)zone->y0, (int)zone->x1, (int)zone->y1);
    logf_both("  calib         %s\r\n",
              (cal && cal->valid) ? "yes (absolute position mode)" : "no");

    log_flush("demo.txt");
}
