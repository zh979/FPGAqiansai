#ifndef BENCH_CPU_H_INCLUDED
#define BENCH_CPU_H_INCLUDED

#include "sd_io.h"

void cpu_conv_leakyrelu(int ch_in, int ch_out, int pad, int stride, int k,
                        int h, int w, const data_t* in, const data_t* weight,
                        const data_t* bias, data_t* out, int act);

typedef void (*bench_draw_fn)(int row, float frac, int done);

typedef void (*bench_repaint_fn)(void);

extern double g_bench_ms_pl;

void bench_run(void* yolo, data_t* in, bench_draw_fn draw,
               bench_repaint_fn repaint);

#endif
