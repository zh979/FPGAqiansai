#pragma GCC optimize(3, "inline")

#include "gimbal.h"
#include <stdio.h>

void gb_init(Gimbal* g, float smooth)
{
    g->virt_w = GB_VIRT_W;
    g->virt_h = GB_VIRT_H;
    if (smooth < 0.05f) smooth = 0.05f;
    if (smooth > 1.0f)  smooth = 1.0f;
    g->smooth = smooth;
    g->sx = g->sy = 0.0f;
    g->has_state = 0;
    g->has_last = 0;
    g->last_nx = g->last_ny = g->last_laser = 0;
    g->last_is_pos = 0;
    g->sent = 0;
    g->repeated = 0;
    g->skipped = 0;
}

void gb_lost(Gimbal* g)
{
    g->has_state = 0;

    g->has_last = 0;

    g->skipped++;
}

static void gb_emit(int nx, int ny, int laser)
{
    printf("X%dY%dL%d\r\n", nx, ny, laser ? 1 : 0);
}

static void gp_emit(int gx, int gy, int laser)
{
    printf("A%dB%dL%d\r\n", gx, gy, laser ? 1 : 0);
}

int gb_send(Gimbal* g, float cx, float cy, int frame_w, int frame_h,
            int laser, int* out_nx, int* out_ny)
{
    if (frame_w <= 0 || frame_h <= 0)
        return 0;

    float fx = cx * (float)g->virt_w / (float)frame_w;
    float fy = cy * (float)g->virt_h / (float)frame_h;

    if (!g->has_state) {
        g->sx = fx;
        g->sy = fy;
        g->has_state = 1;
    } else {
        g->sx += (fx - g->sx) * g->smooth;
        g->sy += (fy - g->sy) * g->smooth;
    }

    int nx = (int)(g->sx + 0.5f);
    int ny = (int)(g->sy + 0.5f);
    if (nx < 0) nx = 0;
    if (ny < 0) ny = 0;
    if (nx > g->virt_w) nx = g->virt_w;
    if (ny > g->virt_h) ny = g->virt_h;

    gb_emit(nx, ny, laser);
    g->sent++;

    g->last_nx    = nx;
    g->last_ny    = ny;
    g->last_laser = laser;
    g->last_is_pos = 0;
    g->has_last   = 1;

    if (out_nx) *out_nx = nx;
    if (out_ny) *out_ny = ny;
    return 1;
}

int gb_repeat(Gimbal* g)
{
    if (!g->has_last)
        return 0;

    if (g->last_is_pos)
        gp_emit(g->last_nx, g->last_ny, g->last_laser);
    else
        gb_emit(g->last_nx, g->last_ny, g->last_laser);
    g->repeated++;
    return 1;
}

int gp_send(Gimbal* g, long gx, long gy, int laser,
            int* out_nx, int* out_ny)
{
    printf("A%ldB%ldL%d\r\n", gx, gy, laser ? 1 : 0);

    g->last_nx    = (int)gx;
    g->last_ny    = (int)gy;
    g->last_laser = laser;
    g->last_is_pos = 1;
    g->has_last   = 1;
    g->sent++;

    if (out_nx) *out_nx = (int)gx;
    if (out_ny) *out_ny = (int)gy;
    return 1;
}
