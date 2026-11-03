#ifndef CALIB_H_INCLUDED
#define CALIB_H_INCLUDED

#define CAL_MAX_POINTS  12

struct CalPoint {
    float px, py;
    float gx, gy;
};

struct Calib {
    float a[3];
    float b[3];
    int   valid;
    float rms_px;
};

void cal_reset(Calib* c);

int cal_fit(Calib* c, const CalPoint* pts, int n);

int cal_map(const Calib* c, float px, float py, float* gx, float* gy);

void cal_print(const Calib* c);

#endif
