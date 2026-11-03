#pragma GCC optimize(3, "inline")

#include "calib.h"
#include <stdio.h>
#include <math.h>

void cal_reset(Calib* c)
{
    for (int i = 0; i < 3; i++) {
        c->a[i] = 0.0f;
        c->b[i] = 0.0f;
    }
    c->valid = 0;
    c->rms_px = 0.0f;
}

static int solve3x3_2rhs(double A[3][3], double r1[3], double r2[3],
                         double x1[3], double x2[3])
{
    for (int col = 0; col < 3; col++) {
        int piv = col;
        double best = fabs(A[col][col]);
        for (int r = col + 1; r < 3; r++) {
            double v = fabs(A[r][col]);
            if (v > best) { best = v; piv = r; }
        }

        if (best < 1e-9)
            return -1;

        if (piv != col) {
            for (int k = 0; k < 3; k++) {
                double t = A[col][k]; A[col][k] = A[piv][k]; A[piv][k] = t;
            }
            double t1 = r1[col]; r1[col] = r1[piv]; r1[piv] = t1;
            double t2 = r2[col]; r2[col] = r2[piv]; r2[piv] = t2;
        }

        for (int r = col + 1; r < 3; r++) {
            double f = A[r][col] / A[col][col];
            if (f == 0.0) continue;
            for (int k = col; k < 3; k++)
                A[r][k] -= f * A[col][k];
            r1[r] -= f * r1[col];
            r2[r] -= f * r2[col];
        }
    }

    for (int i = 2; i >= 0; i--) {
        double s1 = r1[i], s2 = r2[i];
        for (int k = i + 1; k < 3; k++) {
            s1 -= A[i][k] * x1[k];
            s2 -= A[i][k] * x2[k];
        }
        x1[i] = s1 / A[i][i];
        x2[i] = s2 / A[i][i];
    }
    return 0;
}

int cal_fit(Calib* c, const CalPoint* pts, int n)
{
    cal_reset(c);
    if (!pts || n < 3)
        return -1;
    if (n > CAL_MAX_POINTS)
        n = CAL_MAX_POINTS;

    double Spx = 0, Spy = 0, Spx2 = 0, Spy2 = 0, Spxpy = 0;
    double Sgx = 0, Sgy = 0, Spxgx = 0, Spygx = 0, Spxgy = 0, Spygy = 0;

    for (int i = 0; i < n; i++) {
        double px = pts[i].px, py = pts[i].py;
        double gx = pts[i].gx, gy = pts[i].gy;
        Spx   += px;        Spy   += py;
        Spx2  += px * px;   Spy2  += py * py;
        Spxpy += px * py;
        Sgx   += gx;        Sgy   += gy;
        Spxgx += px * gx;   Spygx += py * gx;
        Spxgy += px * gy;   Spygy += py * gy;
    }

    double A[3][3] = {
        { Spx2,  Spxpy, Spx },
        { Spxpy, Spy2,  Spy },
        { Spx,   Spy,   (double)n }
    };
    double r1[3] = { Spxgx, Spygx, Sgx };
    double r2[3] = { Spxgy, Spygy, Sgy };
    double x1[3] = {0,0,0}, x2[3] = {0,0,0};

    if (solve3x3_2rhs(A, r1, r2, x1, x2) != 0)
        return -2;

    for (int i = 0; i < 3; i++) {
        c->a[i] = (float)x1[i];
        c->b[i] = (float)x2[i];
    }
    c->valid = 1;

    double scale_x = sqrt(x1[0]*x1[0] + x1[1]*x1[1]);
    double scale_y = sqrt(x2[0]*x2[0] + x2[1]*x2[1]);
    double scale = (scale_x + scale_y) * 0.5;
    if (scale < 1e-9) scale = 1.0;

    double sse = 0.0;
    for (int i = 0; i < n; i++) {
        double ex = (x1[0]*pts[i].px + x1[1]*pts[i].py + x1[2]) - pts[i].gx;
        double ey = (x2[0]*pts[i].px + x2[1]*pts[i].py + x2[2]) - pts[i].gy;
        sse += ex*ex + ey*ey;
    }
    c->rms_px = (float)(sqrt(sse / n) / scale);
    return 0;
}

int cal_map(const Calib* c, float px, float py, float* gx, float* gy)
{
    if (!c->valid)
        return 0;
    if (gx) *gx = c->a[0]*px + c->a[1]*py + c->a[2];
    if (gy) *gy = c->b[0]*px + c->b[1]*py + c->b[2];
    return 1;
}

void cal_print(const Calib* c)
{
    if (!c->valid) {
        printf("calib: not fitted\r\n");
        return;
    }

    printf("calib fitted, rms = %ld/100 px\r\n", (long)(c->rms_px * 100.0f));
    printf("  paste into code:\r\n");
    printf("  static const Calib CAL = { {%lde-6f, %lde-6f, %lde-6f},\r\n",
           (long)(c->a[0] * 1e6f), (long)(c->a[1] * 1e6f), (long)(c->a[2] * 1e6f));
    printf("                             {%lde-6f, %lde-6f, %lde-6f}, 1, 0.0f };\r\n",
           (long)(c->b[0] * 1e6f), (long)(c->b[1] * 1e6f), (long)(c->b[2] * 1e6f));
}
