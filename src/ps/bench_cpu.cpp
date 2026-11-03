#pragma GCC optimize(3, "Ofast", "inline")

#include "bench_cpu.h"
#include "basic_op.h"
#include "yolo4_tiny.h"
#include "xtime_l.h"
#include "xil_cache.h"
#include <cstdio>
#include <cmath>

double g_bench_ms_pl = 0.0;

static unsigned long long g_bench_draw_ticks = 0;

static inline data_t q79_sat(int v)
{
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    return (data_t)v;
}

static inline int rnd_shift9(int acc)
{
    return (acc + 256) >> 9;
}

void cpu_conv_leakyrelu(int ch_in, int ch_out, int pad, int stride, int k,
                        int h, int w, const data_t* in, const data_t* weight,
                        const data_t* bias, data_t* out, int act)
{
    const int oh = (stride > 0) ? (h / stride) : h;
    const int ow = (stride > 0) ? (w / stride) : w;

    for (int m = 0; m < ch_out; m++) {
        const data_t* wm = weight + (long)m * ch_in * k * k;
        const int b = (int)bias[m];

        for (int r = 0; r < oh; r++) {
            for (int c = 0; c < ow; c++) {
                int acc = b << 9;

                for (int n = 0; n < ch_in; n++) {
                    const data_t* wn = wm + (long)n * k * k;
                    const data_t* ip = in + (long)n * h * w;

                    for (int i = 0; i < k; i++) {
                        const int y = r * stride + i - pad;
                        if (y < 0 || y >= h) continue;
                        for (int j = 0; j < k; j++) {
                            const int x = c * stride + j - pad;
                            if (x < 0 || x >= w) continue;
                            acc += (int)ip[(long)y * w + x] * (int)wn[i * k + j];
                        }
                    }
                }

                int v = rnd_shift9(acc);

                if (act == 1 && v < 0) {
                    v = (v * 51 + 256) >> 9;
                }

                out[((long)m * oh + r) * ow + c] = q79_sat(v);
            }
        }
    }
}

static void cpu_conv_range(int ch_in, int ch_out, int pad, int stride, int k,
                           int h, int w, const data_t* in,
                           const data_t* weight, const data_t* bias,
                           data_t* out, int act, int m0, int m1,
                           bench_repaint_fn repaint)
{
    const int oh = (stride > 0) ? (h / stride) : h;
    const int ow = (stride > 0) ? (w / stride) : w;

    for (int m = m0; m < m1; m++) {
        const data_t* wm = weight + (long)m * ch_in * k * k;
        const int b = (int)bias[m];

        for (int r = 0; r < oh; r++) {
            if (repaint && (r & 15) == 0) {
                XTime rp0, rp1;
                XTime_GetTime(&rp0);
                repaint();
                XTime_GetTime(&rp1);
                g_bench_draw_ticks += (rp1 - rp0);
            }
            for (int c = 0; c < ow; c++) {
                int acc = b << 9;
                for (int n = 0; n < ch_in; n++) {
                    const data_t* wn = wm + (long)n * k * k;
                    const data_t* ip = in + (long)n * h * w;
                    for (int i = 0; i < k; i++) {
                        const int y = r * stride + i - pad;
                        if (y < 0 || y >= h) continue;
                        for (int j = 0; j < k; j++) {
                            const int x = c * stride + j - pad;
                            if (x < 0 || x >= w) continue;
                            acc += (int)ip[(long)y * w + x] * (int)wn[i * k + j];
                        }
                    }
                }
                int v = rnd_shift9(acc);
                if (act == 1 && v < 0)
                    v = (v * 51 + 256) >> 9;
                out[((long)m * oh + r) * ow + c] = q79_sat(v);
            }
        }
    }
}

void bench_run(void* yolo_v, data_t* in, bench_draw_fn draw,
               bench_repaint_fn repaint)
{
    Yolo4_Tiny* yolo = (Yolo4_Tiny*)yolo_v;
    BasicConv* L1 = yolo->backbone->basic_conv1;

    const int ch_in = L1->ch_in, ch_out = L1->ch_out;
    const int h = L1->h, w = L1->w, k = L1->k, s = L1->s, p = L1->p;
    const int oh = h / s, ow = w / s;
    const long osize = (long)ch_out * oh * ow;

    static data_t out_pl[32 * 208 * 208];
    static data_t out_cpu[32 * 208 * 208];

    printf("\r\n");
    printf("==================================================\r\n");
    printf("  PL ACCELERATOR  vs  ARM SOFTWARE\r\n");
    printf("  (same layer, same data, same numeric format)\r\n");
    printf("==================================================\r\n");
    printf("  Layer : BasicConv1  %dx%dx%d -> %d ch, %dx%d kernel, stride %d\r\n",
           ch_in, h, w, ch_out, k, k, s);
    printf("  MACs  : %ld\r\n",
           (long)ch_out * ch_in * k * k * oh * ow);
    printf("  Format: Q7.9 fixed point (both sides)\r\n");
    printf("\r\n");

    XTime t0, t1;
    double ms_pl, ms_cpu;

    printf("  [1/2] PL accelerator ... ");

    Xil_DCacheFlush();
    XTime_GetTime(&t0);
    conv_leakyrelu(ch_in, ch_out, p, s, k, h, w,
                   in, L1->weight, L1->bias, out_pl, 1);
    XTime_GetTime(&t1);

    Xil_DCacheInvalidateRange((unsigned int)(unsigned long)out_pl,
                              (unsigned int)(osize * sizeof(data_t)));
    ms_pl = (double)(t1 - t0) * 1000.0 / (double)COUNTS_PER_SECOND;
    g_bench_ms_pl = ms_pl;
    printf("%.2f ms\r\n", ms_pl);

    if (draw && repaint) {
        XTime a0, an;
        XTime_GetTime(&a0);
        const unsigned long long adur =
            (unsigned long long)COUNTS_PER_SECOND * 3 / 25;
        while (1) {
            XTime_GetTime(&an);
            unsigned long long el = an - a0;
            if (el >= adur)
                break;
            draw(0, (float)((double)el / (double)adur), 0);
        }
    }
    if (draw)
        draw(0, 1.0f, 1);

    if (repaint) {
        XTime g0, gn;
        XTime_GetTime(&g0);
        do {
            repaint();
            XTime_GetTime(&gn);
        } while (gn - g0 < (unsigned long long)(COUNTS_PER_SECOND * 3 / 2));
    }

    printf("  [2/2] ARM software   ... (please wait)\r\n");

    const int BENCH_SEG = 32;
    g_bench_draw_ticks = 0;
    XTime_GetTime(&t0);
    for (int seg = 0; seg < BENCH_SEG; seg++) {
        int m0 = (int)((long)ch_out * seg / BENCH_SEG);
        int m1 = (int)((long)ch_out * (seg + 1) / BENCH_SEG);
        if (m1 > m0)
            cpu_conv_range(ch_in, ch_out, p, s, k, h, w,
                           in, L1->weight, L1->bias, out_cpu, 1, m0, m1,
                           repaint);
        if (draw) {
            XTime rp0, rp1;
            XTime_GetTime(&rp0);
            draw(1, (float)(seg + 1) / (float)BENCH_SEG,
                 (seg + 1 == BENCH_SEG) ? 1 : 0);
            XTime_GetTime(&rp1);
            g_bench_draw_ticks += (rp1 - rp0);
        }
    }
    XTime_GetTime(&t1);

    unsigned long long cpu_ticks = (t1 - t0);
    if (cpu_ticks > g_bench_draw_ticks)
        cpu_ticks -= g_bench_draw_ticks;
    ms_cpu = (double)cpu_ticks * 1000.0 / (double)COUNTS_PER_SECOND;
    printf("        (excluded %.1f ms of screen drawing)\r\n",
           (double)g_bench_draw_ticks * 1000.0 / (double)COUNTS_PER_SECOND);
    printf("                      %.2f ms\r\n", ms_cpu);

    double sum_abs = 0.0, sum_err = 0.0;
    int    max_err = 0;

    for (long i = 0; i < osize; i++) {
        int a = (int)out_pl[i], bb = (int)out_cpu[i];
        int d = a - bb;
        if (d < 0) d = -d;
        int aa = (a < 0) ? -a : a;
        sum_abs += (double)aa;
        sum_err += (double)d;
        if (d > max_err) max_err = d;
    }

    double rel_mean = (sum_abs > 1.0) ? (sum_err / sum_abs * 100.0) : 0.0;

    double rel_max  = (double)max_err / 32768.0 * 100.0;

    printf("\r\n");
    printf("  -------------- RESULT --------------\r\n");
    printf("  PL  accelerator : %9.2f ms\r\n", ms_pl);
    printf("  ARM software    : %9.2f ms\r\n", ms_cpu);
    if (ms_pl > 0.001)
        printf("  SPEEDUP         : %9.1f x\r\n", ms_cpu / ms_pl);
    printf("\r\n");
    printf("  Numerical agreement (different rounding paths):\r\n");
    printf("    mean relative error : %.3f %%\r\n", rel_mean);
    printf("    max  point error    : %.3f %% of full scale\r\n", rel_max);
    if (rel_mean < 2.0)
        printf("  --> equivalent within fixed-point tolerance\r\n");
    else
        printf("  --> !! error too large, check CPU reference\r\n");
    printf("==================================================\r\n\r\n");
}
