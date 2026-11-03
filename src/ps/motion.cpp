#pragma GCC optimize(3, "inline")

#include "motion.h"

static unsigned char g_ref[MO_CELLS];
static int g_have_ref = 0;

static int g_xt[MO_GRID_W];
static int g_yt[MO_GRID_H];
static int g_tab_w = -1, g_tab_h = -1;

void motion_reset(void)
{
    g_have_ref = 0;
}

static int g_ignore_bottom = 0;

void motion_set_ignore_bottom(int rows)
{
    if (rows < 0)
        rows = 0;
    if (rows == g_ignore_bottom)
        return;
    g_ignore_bottom = rows;
    g_tab_h = -1;
    g_have_ref = 0;
}

static void build_tabs(int w, int h)
{
    if (g_tab_w == w && g_tab_h == h)
        return;

    int h_eff = h - g_ignore_bottom;
    if (h_eff < MO_GRID_H)
        h_eff = h;

    for (int i = 0; i < MO_GRID_W; i++)
        g_xt[i] = (int)(((long)i * 2 + 1) * w / (2L * MO_GRID_W));
    for (int i = 0; i < MO_GRID_H; i++)
        g_yt[i] = (int)(((long)i * 2 + 1) * h_eff / (2L * MO_GRID_H));
    g_tab_w = w;
    g_tab_h = h;
}

int motion_detect(const unsigned char* fb, int w, int h, int stride,
                  int cell_thres, int cell_count, MotionResult* res)
{
    if (!fb || !res || w <= 0 || h <= 0)
        return 0;
    if (stride <= 0)
        stride = w * 3;

    build_tabs(w, h);

    res->moving = 0;
    res->changed_cells = 0;
    res->energy = 0;
    res->bx0 = MO_GRID_W; res->by0 = MO_GRID_H;
    res->bx1 = -1;        res->by1 = -1;

    long sum_diff = 0;
    int changed = 0;
    int bx0 = MO_GRID_W, by0 = MO_GRID_H, bx1 = -1, by1 = -1;

    for (int r = 0; r < MO_GRID_H; r++) {
        const unsigned char* row = fb + (long)g_yt[r] * stride;
        unsigned char* ref = &g_ref[r * MO_GRID_W];

        for (int c = 0; c < MO_GRID_W; c++) {
            const unsigned char* p = row + (long)g_xt[c] * 3;

            int gray = (p[0] + 2 * p[1] + p[2]) >> 2;

            int d = gray - (int)ref[c];
            if (d < 0) d = -d;
            sum_diff += d;

            if (d > cell_thres) {
                changed++;
                if (c < bx0) bx0 = c;
                if (c > bx1) bx1 = c;
                if (r < by0) by0 = r;
                if (r > by1) by1 = r;
            }

            ref[c] = (unsigned char)((int)ref[c] + ((gray - (int)ref[c]) >> 3));
        }
    }

    res->changed_cells = changed;
    res->energy = (int)(sum_diff / MO_CELLS);

    if (!g_have_ref) {
        for (int r = 0; r < MO_GRID_H; r++) {
            const unsigned char* row = fb + (long)g_yt[r] * stride;
            for (int c = 0; c < MO_GRID_W; c++) {
                const unsigned char* p = row + (long)g_xt[c] * 3;
                g_ref[r * MO_GRID_W + c] =
                    (unsigned char)((p[0] + 2 * p[1] + p[2]) >> 2);
            }
        }
        g_have_ref = 1;
        res->moving = 1;
        res->changed_cells = MO_CELLS;
        res->bx0 = 0; res->by0 = 0;
        res->bx1 = MO_GRID_W - 1; res->by1 = MO_GRID_H - 1;
        return 1;
    }

    if (changed >= cell_count) {
        res->moving = 1;
        res->bx0 = bx0; res->by0 = by0;
        res->bx1 = bx1; res->by1 = by1;
    }
    return res->moving;
}
