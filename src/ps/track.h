#ifndef TRACK_H_INCLUDED
#define TRACK_H_INCLUDED

#include "yolo_post.h"

struct Kalman2D {
    float x[4];
    float P[4][4];
    int   inited;

    float q_pos;
    float q_vel;
    float r_meas;
};

void kf_init(Kalman2D* kf);

void kf_start(Kalman2D* kf, float px, float py);

void kf_predict(Kalman2D* kf, float dt);

void kf_update(Kalman2D* kf, float px, float py);

void kf_peek(const Kalman2D* kf, float dt, float* px, float* py);

#define TRK_CLASS_ANY   (-1)

#define TRK_MAX_SKIP    16

struct Tracker {
    Kalman2D kf;
    int      active;
    int      class_id;
    int      want_class;
    float    w, h;
    int      lost;
    int      coast_max;
    int      hits;

    int      uid;
    float    size_lpf;
    float    gate;

    float    skip_x[TRK_MAX_SKIP];
    float    skip_y[TRK_MAX_SKIP];

    float    skip_vx[TRK_MAX_SKIP];
    float    skip_vy[TRK_MAX_SKIP];
    unsigned long skip_t[TRK_MAX_SKIP];

    int      skip_n;
    int      skip_head;
    float    skip_r;

    unsigned long now_ms;
    unsigned long skip_max_age_ms;
    float    vel_deadband;
};

struct TrackOut {
    int   valid;
    int   measured;
    float cx, cy;
    float raw_cx, raw_cy;
    float vx, vy;
    float w, h;
    int   class_id;
    int   lost;
};

void trk_init(Tracker* t, int want_class, int coast_max, float gate_px);

void trk_release(Tracker* t);

void trk_mark_done(Tracker* t, float x, float y, float vx, float vy,
                   float radius_px);

void trk_set_time(Tracker* t, unsigned long now_ms);

void trk_clear_done(Tracker* t);

void trk_update(Tracker* t, const YoloBox* dets, int nd,
                float dt, float lead_sec, TrackOut* out);

void trk_coast(Tracker* t, float dt, float lead_sec, TrackOut* out);

#endif
