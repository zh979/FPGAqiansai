#pragma GCC optimize(3, "inline")

#include "yolo_pre.h"

#define NS  YPRE_NET_SIZE
#define NPX (YPRE_NET_SIZE * YPRE_NET_SIZE)

static short g_q[256];
static int   g_q_ready = 0;

static void yp_build_lut(void)
{
    for (int v = 0; v < 256; v++)
        g_q[v] = (short)((v * 512 + 127) / 255);
    g_q_ready = 1;
}

#define YP_FBITS  4
#define YP_FONE   (1 << YP_FBITS)

static int g_xtab[NS];
static int g_ytab[NS];
static int g_xi[NS];
static int g_yi[NS];
static int g_xfrac[NS];
static int g_yfrac[NS];
static int g_cache_valid = 0;
static int g_c_rx, g_c_ry, g_c_rw, g_c_rh;

static void yp_build_tabs(int roi_x, int roi_y, int roi_w, int roi_h)
{
    if (g_cache_valid && roi_x == g_c_rx && roi_y == g_c_ry &&
        roi_w == g_c_rw && roi_h == g_c_rh)
        return;

    for (int i = 0; i < NS; i++) {
        g_xtab[i] = roi_x + (int)(((long)i * 2 + 1) * roi_w / (2L * NS));
        g_ytab[i] = roi_y + (int)(((long)i * 2 + 1) * roi_h / (2L * NS));

        long sx = ((long)(2 * i + 1) * roi_w * YP_FONE) / (2L * NS) - (YP_FONE / 2);
        long sy = ((long)(2 * i + 1) * roi_h * YP_FONE) / (2L * NS) - (YP_FONE / 2);
        if (sx < 0) sx = 0;
        if (sy < 0) sy = 0;
        g_xi[i]    = roi_x + (int)(sx >> YP_FBITS);
        g_yi[i]    = roi_y + (int)(sy >> YP_FBITS);
        g_xfrac[i] = (int)(sx & (YP_FONE - 1));
        g_yfrac[i] = (int)(sy & (YP_FONE - 1));
    }
    g_c_rx = roi_x; g_c_ry = roi_y; g_c_rw = roi_w; g_c_rh = roi_h;
    g_cache_valid = 1;
}

int yolo_preprocess(const unsigned char* src,
                    int src_w, int src_h, int src_stride,
                    int roi_x, int roi_y, int roi_w, int roi_h,
                    int mode,
                    short* dst)
{
    if (src == 0 || dst == 0)
        return -1;
    if (src_w <= 0 || src_h <= 0)
        return -2;

    if (src_stride <= 0)
        src_stride = src_w * 3;

    if (roi_w <= 0) roi_w = src_w;
    if (roi_h <= 0) roi_h = src_h;
    if (roi_x < 0) roi_x = 0;
    if (roi_y < 0) roi_y = 0;
    if (roi_x >= src_w) roi_x = src_w - 1;
    if (roi_y >= src_h) roi_y = src_h - 1;
    if (roi_x + roi_w > src_w) roi_w = src_w - roi_x;
    if (roi_y + roi_h > src_h) roi_h = src_h - roi_y;
    if (roi_w <= 0 || roi_h <= 0)
        return -3;

    if (!g_q_ready)
        yp_build_lut();
    yp_build_tabs(roi_x, roi_y, roi_w, roi_h);

#if YPRE_SRC_BGR
    const int O_R = 2, O_G = 1, O_B = 0;
#else
    const int O_R = 0, O_G = 1, O_B = 2;
#endif

    short* dR = dst;
    short* dG = dst + NPX;
    short* dB = dst + 2 * NPX;

    int step_x = roi_w / NS; if (step_x < 1) step_x = 1;
    int step_y = roi_h / NS; if (step_y < 1) step_y = 1;

    if (mode == YPRE_BILINEAR) {
        for (int r = 0; r < NS; r++) {
            int y0 = g_yi[r];
            if (y0 >= src_h) y0 = src_h - 1;
            int y1 = y0 + 1;
            if (y1 >= src_h) y1 = src_h - 1;
            const int wy = g_yfrac[r];
            const int iwy = YP_FONE - wy;

            const unsigned char* row0 = src + (long)y0 * src_stride;
            const unsigned char* row1 = src + (long)y1 * src_stride;
            short* pR = dR + r * NS;
            short* pG = dG + r * NS;
            short* pB = dB + r * NS;

            for (int c = 0; c < NS; c++) {
                int x0 = g_xi[c];
                if (x0 >= src_w) x0 = src_w - 1;
                int x1 = x0 + 1;
                if (x1 >= src_w) x1 = src_w - 1;
                const int wx = g_xfrac[c];
                const int iwx = YP_FONE - wx;

                const unsigned char* a = row0 + x0 * 3;
                const unsigned char* b = row0 + x1 * 3;
                const unsigned char* e = row1 + x0 * 3;
                const unsigned char* f = row1 + x1 * 3;

                const int tR = (a[O_R] * iwx + b[O_R] * wx) * iwy
                             + (e[O_R] * iwx + f[O_R] * wx) * wy;
                const int tG = (a[O_G] * iwx + b[O_G] * wx) * iwy
                             + (e[O_G] * iwx + f[O_G] * wx) * wy;
                const int tB = (a[O_B] * iwx + b[O_B] * wx) * iwy
                             + (e[O_B] * iwx + f[O_B] * wx) * wy;

                pR[c] = g_q[(tR + 128) >> 8];
                pG[c] = g_q[(tG + 128) >> 8];
                pB[c] = g_q[(tB + 128) >> 8];
            }
        }
        return 0;
    }

    for (int r = 0; r < NS; r++) {
        const int sy = g_ytab[r];
        const unsigned char* row0 = src + (long)sy * src_stride;

        if (mode == YPRE_BOX2X2) {
            int sy2 = sy + step_y;
            if (sy2 >= src_h) sy2 = src_h - 1;
            const unsigned char* row1 = src + (long)sy2 * src_stride;

            for (int c = 0; c < NS; c++) {
                const int sx = g_xtab[c];
                int sx2 = sx + step_x;
                if (sx2 >= src_w) sx2 = src_w - 1;

                const unsigned char* a = row0 + sx * 3;
                const unsigned char* b = row0 + sx2 * 3;
                const unsigned char* e = row1 + sx * 3;
                const unsigned char* f = row1 + sx2 * 3;

                const int vr = (a[O_R] + b[O_R] + e[O_R] + f[O_R] + 2) >> 2;
                const int vg = (a[O_G] + b[O_G] + e[O_G] + f[O_G] + 2) >> 2;
                const int vb = (a[O_B] + b[O_B] + e[O_B] + f[O_B] + 2) >> 2;

                const int o = r * NS + c;
                dR[o] = g_q[vr];
                dG[o] = g_q[vg];
                dB[o] = g_q[vb];
            }
        } else {
            for (int c = 0; c < NS; c++) {
                const unsigned char* p = row0 + g_xtab[c] * 3;
                const int o = r * NS + c;
                dR[o] = g_q[p[O_R]];
                dG[o] = g_q[p[O_G]];
                dB[o] = g_q[p[O_B]];
            }
        }
    }
    return 0;
}

void yolo_map_box_to_frame(float* top, float* left, float* bottom, float* right,
                           int roi_x, int roi_y, int roi_w, int roi_h)
{
    if (top)    *top    += (float)roi_y;
    if (bottom) *bottom += (float)roi_y;
    if (left)   *left   += (float)roi_x;
    if (right)  *right  += (float)roi_x;

    (void)roi_w;
    (void)roi_h;
}
