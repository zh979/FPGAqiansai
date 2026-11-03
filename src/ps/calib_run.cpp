#include "calib_run.h"
#include "calib.h"
#include "gimbal.h"
#include "yolo_pre.h"
#include "xparameters.h"
#include "xuartps_hw.h"
#include "xil_cache.h"
#include "xtime_l.h"
#include "sleep.h"
#include "xstatus.h"
#include <stdio.h>
#include <math.h>

#define UART_BASE   XPAR_PS7_UART_0_BASEADDR

extern int SD_Transfer_write(char* FileName, u32 SourceAddress, u32 ByteLength);

#define CAL_OUT_FILE   "calib.txt"
static char g_cal_txt[2048];

#define JOG_FINE     4
#define JOG_COARSE  40

#define BLINK_MS    2500

static const float CAL_PTS[][2] = {
    {0.50f, 0.50f},
    {0.20f, 0.20f}, {0.80f, 0.20f},
    {0.20f, 0.80f}, {0.80f, 0.80f},
    {0.50f, 0.20f}, {0.50f, 0.80f},
    {0.20f, 0.50f}, {0.80f, 0.50f},
};
#define N_CAL_PTS  ((int)(sizeof(CAL_PTS) / sizeof(CAL_PTS[0])))

static int uart_getc_nb(void)
{
    if (XUartPs_IsReceiveData(UART_BASE))
        return (int)XUartPs_RecvByte(UART_BASE);
    return 0;
}

static void draw_cross(unsigned char* fb, int w, int h, int stride,
                       int cx, int cy, unsigned char r,
                       unsigned char g, unsigned char b)
{
#if YPRE_SRC_BGR
    const int O_R = 2, O_G = 1, O_B = 0;
#else
    const int O_R = 0, O_G = 1, O_B = 2;
#endif
    const int ARM = 26, GAP = 5, TH = 2;

    for (int d = GAP; d <= ARM; d++) {
        for (int t = -TH; t <= TH; t++) {
            int xs[4] = { cx - d, cx + d, cx + t, cx + t };
            int ys[4] = { cy + t, cy + t, cy - d, cy + d };
            for (int k = 0; k < 4; k++) {
                int x = xs[k], y = ys[k];
                if (x < 0 || y < 0 || x >= w || y >= h)
                    continue;
                unsigned char* p = fb + (unsigned int)y * stride + x * 3;
                p[O_R] = r; p[O_G] = g; p[O_B] = b;
            }
        }
    }
}

static void emit_pos(long gx, long gy, int laser)
{
    printf("A%ldB%ldL%d\r\n", gx, gy, laser ? 1 : 0);
}

void calib_run(unsigned int fb_cap, int w, int h)
{
    const int stride = w * 3;
    const unsigned int fb_bytes = (unsigned int)stride * (unsigned int)h;

    CalPoint pts[CAL_MAX_POINTS];
    int n_pts = 0;

    long gx = 0, gy = 0;
    int  idx = 0;
    int  laser = 1;
    int  coarse = 0;

    printf("\r\n=== Hand-eye calibration ===\r\n");
    printf("frame %dx%d, %d points to collect\r\n", w, h, N_CAL_PTS);
    printf("\r\n");
    printf("BEFORE YOU START:\r\n");
    printf("  1. aim the gimbal roughly at the middle of the belt by hand\r\n");
    printf("  2. make sure the laser is mounted and powered\r\n");
    printf("  3. nobody downstream of the beam\r\n");
    printf("\r\n");
    printf("keys:\r\n");
    printf("  w / s     jog along  (motor 1)\r\n");
    printf("  a / d     jog across (motor 2)\r\n");
    printf("  f         toggle fine(%d) / coarse(%d) step\r\n",
           JOG_FINE, JOG_COARSE);
    printf("  r         record this point, go to next\r\n");
    printf("  n         skip this point\r\n");
    printf("  z         re-zero the gimbal here\r\n");
    printf("  SPACE     laser on/off\r\n");
    printf("  c         clear all recorded points\r\n");
    printf("  q         finish and fit\r\n");
    printf("\r\n");
    printf("Move the RED DOT onto the cross on screen, then press r.\r\n");
    printf("\r\n");

    printf("Z\r\n");
    usleep(300000);

    XTime t_blink, t_ka, now;
    XTime_GetTime(&t_blink);
    t_ka = t_blink;

    int redraw = 1;

    while (1) {
        int cx = (int)(CAL_PTS[idx][0] * (float)w);
        int cy = (int)(CAL_PTS[idx][1] * (float)h);

        Xil_DCacheInvalidateRange(fb_cap, fb_bytes);
        draw_cross((unsigned char*)fb_cap, w, h, stride, cx, cy, 0, 255, 0);
        Xil_DCacheFlushRange(fb_cap, fb_bytes);

        if (redraw) {
            printf("[pt %d/%d] cross at px(%d,%d)   pulse(%ld,%ld)   "
                   "step=%s  laser=%s  recorded=%d\r\n",
                   idx + 1, N_CAL_PTS, cx, cy, gx, gy,
                   coarse ? "COARSE" : "fine",
                   laser ? "ON" : "off", n_pts);
            redraw = 0;
        }

        XTime_GetTime(&now);

        if (laser &&
            (now - t_blink) > (XTime)(COUNTS_PER_SECOND / 1000 * BLINK_MS)) {
            t_blink = now;
            emit_pos(gx, gy, 0);
            usleep(120000);
        }

        if ((now - t_ka) > (XTime)(COUNTS_PER_SECOND / 7)) {
            t_ka = now;
            emit_pos(gx, gy, laser);
        }

        int c = uart_getc_nb();
        if (c == 0) {
            usleep(5000);
            continue;
        }

        long step = coarse ? JOG_COARSE : JOG_FINE;

        switch (c) {
        case 'w': gy += step; redraw = 1; break;
        case 's': gy -= step; redraw = 1; break;
        case 'a': gx -= step; redraw = 1; break;
        case 'd': gx += step; redraw = 1; break;

        case 'f': case 'F':
            coarse = !coarse; redraw = 1; break;

        case ' ':
            laser = !laser;
            emit_pos(gx, gy, laser);
            redraw = 1;
            break;

        case 'z': case 'Z':
            printf("Z\r\n");
            gx = gy = 0;
            usleep(300000);
            printf("  re-zeroed\r\n");
            redraw = 1;
            break;

        case 'r': case 'R':
            if (n_pts >= CAL_MAX_POINTS) {
                printf("  !! point buffer full (%d)\r\n", CAL_MAX_POINTS);
                break;
            }
            pts[n_pts].px = (float)cx;
            pts[n_pts].py = (float)cy;
            pts[n_pts].gx = (float)gx;
            pts[n_pts].gy = (float)gy;
            n_pts++;
            printf("  recorded #%d: px(%d,%d) -> pulse(%ld,%ld)\r\n",
                   n_pts, cx, cy, gx, gy);
            if (idx + 1 < N_CAL_PTS) {
                idx++;
            } else {
                printf("  all %d points visited, press q to fit\r\n",
                       N_CAL_PTS);
            }
            redraw = 1;
            break;

        case 'n': case 'N':
            if (idx + 1 < N_CAL_PTS) idx++;
            redraw = 1;
            break;

        case 'c': case 'C':
            n_pts = 0;
            idx = 0;
            printf("  cleared\r\n");
            redraw = 1;
            break;

        case 'q': case 'Q':
            goto done;

        default:
            break;
        }
    }

done:
    emit_pos(gx, gy, 0);
    usleep(100000);
    emit_pos(gx, gy, 0);

    printf("\r\n=== fitting %d points ===\r\n", n_pts);
    if (n_pts < 3) {
        printf("need at least 3 points (4+ recommended), got %d\r\n", n_pts);
        return;
    }

    Calib cal;
    cal_reset(&cal);

    int fit_rc = cal_fit(&cal, pts, n_pts);

    {
        int n = snprintf(g_cal_txt, sizeof(g_cal_txt),
            "hand-eye calibration\r\n"
            "points=%d  fit_rc=%d  residual=%.3f px  (1 px = 0.5375 mm)\r\n"
            "\r\n"
            "raw points (px_x px_y  pulse_x pulse_y):\r\n",
            n_pts, fit_rc, cal.rms_px);
        for (int i = 0; i < n_pts && n < (int)sizeof(g_cal_txt) - 96; i++) {
            n += snprintf(g_cal_txt + n, sizeof(g_cal_txt) - n,
                          "  %4.0f %4.0f  %6.0f %6.0f\r\n",
                          pts[i].px, pts[i].py, pts[i].gx, pts[i].gy);
        }
        if (fit_rc == 0 && n < (int)sizeof(g_cal_txt) - 256) {
            n += snprintf(g_cal_txt + n, sizeof(g_cal_txt) - n,
                "\r\nstatic const Calib CAL = {\r\n"
                "    { %.6ff, %.6ff, %.6ff },\r\n"
                "    { %.6ff, %.6ff, %.6ff },\r\n"
                "    1, %.3ff\r\n"
                "};\r\n",
                cal.a[0], cal.a[1], cal.a[2],
                cal.b[0], cal.b[1], cal.b[2], cal.rms_px);
        }
        if (SD_Transfer_write((char*)CAL_OUT_FILE,
                              (u32)(unsigned long)g_cal_txt,
                              (u32)n) == XST_SUCCESS)
            printf("wrote %s to SD card (%d bytes)\r\n", CAL_OUT_FILE, n);
        else
            printf("!! could not write %s\r\n", CAL_OUT_FILE);
    }

    if (fit_rc != 0) {
        printf("\r\nFIT FAILED (rc=%d): %s\r\n", fit_rc,
               (fit_rc == -1) ? "need at least 3 points"
                              : "points degenerate (collinear or coincident)");
        printf("raw points were still saved to %s\r\n", CAL_OUT_FILE);
        return;
    }

    cal_print(&cal);

    printf("\r\n--- paste into main.cpp, LOOP_MODE 4 block ---\r\n");
    printf("static const Calib CAL = {\r\n");
    printf("    { %.6ff, %.6ff, %.6ff },\r\n",
           cal.a[0], cal.a[1], cal.a[2]);
    printf("    { %.6ff, %.6ff, %.6ff },\r\n",
           cal.b[0], cal.b[1], cal.b[2]);
    printf("    1, %.3ff\r\n", cal.rms_px);
    printf("};\r\n");
    printf("--- end ---\r\n\r\n");

    printf("residual %.2f px", cal.rms_px);
    if (cal.rms_px < 5.0f)
        printf("  -- good\r\n");
    else if (cal.rms_px < 15.0f)
        printf("  -- usable\r\n");
    else
        printf("  -- TOO HIGH, suspect a mis-recorded point; redo with c\r\n");

    printf("\r\nnote: the gimbal zero is where you pressed Z.\r\n");
    printf("      these constants are only valid for that zero --\r\n");
    printf("      re-run calibration if the gimbal is ever re-homed\r\n");
    printf("      or the camera/laser is moved.\r\n");

    while (1)
        ;
}
