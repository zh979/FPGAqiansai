#pragma GCC optimize(3, "inline")

#include "track.h"
#include <math.h>

void kf_init(Kalman2D* kf)
{
    for (int i = 0; i < 4; i++) {
        kf->x[i] = 0.0f;
        for (int j = 0; j < 4; j++)
            kf->P[i][j] = 0.0f;
    }
    kf->inited = 0;

    kf->q_pos  = 1.0f;
    kf->q_vel  = 4.0f;
    kf->r_meas = 25.0f;
}

void kf_start(Kalman2D* kf, float px, float py)
{
    kf->x[0] = px;
    kf->x[1] = py;
    kf->x[2] = 0.0f;
    kf->x[3] = 0.0f;

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            kf->P[i][j] = 0.0f;

    kf->P[0][0] = 16.0f;
    kf->P[1][1] = 16.0f;
    kf->P[2][2] = 400.0f;
    kf->P[3][3] = 400.0f;

    kf->inited = 1;
}

void kf_predict(Kalman2D* kf, float dt)
{
    if (!kf->inited)
        return;
    if (dt < 0.0f)
        dt = 0.0f;

    if (dt > 1.0f)
        dt = 1.0f;

    kf->x[0] += kf->x[2] * dt;
    kf->x[1] += kf->x[3] * dt;

    const float q_p = kf->q_pos * dt;
    const float q_v = kf->q_vel * dt;

    for (int k = 0; k < 2; k++) {
        const int ip = k;
        const int iv = k + 2;

        const float ppp = kf->P[ip][ip];
        const float ppv = kf->P[ip][iv];
        const float pvp = kf->P[iv][ip];
        const float pvv = kf->P[iv][iv];

        kf->P[ip][ip] = ppp + dt * (ppv + pvp) + dt * dt * pvv + q_p;
        kf->P[ip][iv] = ppv + dt * pvv;
        kf->P[iv][ip] = pvp + dt * pvv;
        kf->P[iv][iv] = pvv + q_v;
    }
}

void kf_update(Kalman2D* kf, float px, float py)
{
    if (!kf->inited) {
        kf_start(kf, px, py);
        return;
    }

    const float z[2] = { px, py };

    for (int k = 0; k < 2; k++) {
        const int ip = k;
        const int iv = k + 2;

        const float S = kf->P[ip][ip] + kf->r_meas;
        if (S <= 0.0f)
            continue;
        const float inv = 1.0f / S;

        const float Kp = kf->P[ip][ip] * inv;
        const float Kv = kf->P[iv][ip] * inv;

        const float innov = z[k] - kf->x[ip];
        kf->x[ip] += Kp * innov;
        kf->x[iv] += Kv * innov;

        const float ppp = kf->P[ip][ip];
        const float ppv = kf->P[ip][iv];
        const float pvp = kf->P[iv][ip];
        const float pvv = kf->P[iv][iv];

        kf->P[ip][ip] = ppp - Kp * ppp;
        kf->P[ip][iv] = ppv - Kp * ppv;
        kf->P[iv][ip] = pvp - Kv * ppp;
        kf->P[iv][iv] = pvv - Kv * ppv;
    }
}

void kf_peek(const Kalman2D* kf, float dt, float* px, float* py)
{
    if (px) *px = kf->x[0] + kf->x[2] * dt;
    if (py) *py = kf->x[1] + kf->x[3] * dt;
}

void trk_init(Tracker* t, int want_class, int coast_max, float gate_px)
{
    kf_init(&t->kf);
    t->active     = 0;
    t->class_id   = -1;
    t->want_class = want_class;
    t->w = t->h   = 0.0f;
    t->lost       = 0;
    t->coast_max  = coast_max > 0 ? coast_max : 4;
    t->hits       = 0;
    t->size_lpf   = 0.4f;
    t->uid        = 0;
    t->skip_n     = 0;
    t->skip_head  = 0;
    t->skip_r     = 40.0f;
    t->now_ms     = 0;
    t->skip_max_age_ms = 4000;
    t->vel_deadband    = 30.0f;
    for (int i = 0; i < TRK_MAX_SKIP; i++) {
        t->skip_vx[i] = t->skip_vy[i] = 0.0f;
        t->skip_t[i]  = 0;
    }
    t->gate       = gate_px > 0.0f ? gate_px : 200.0f;
}

static void fill_out(Tracker* t, TrackOut* out, int measured, float lead_sec)
{
    out->valid    = 1;
    out->measured = measured;
    out->raw_cx   = t->kf.x[0];
    out->raw_cy   = t->kf.x[1];
    out->vx       = t->kf.x[2];
    out->vy       = t->kf.x[3];
    kf_peek(&t->kf, lead_sec, &out->cx, &out->cy);
    out->w        = t->w;
    out->h        = t->h;
    out->class_id = t->class_id;
    out->lost     = t->lost;
}

static void fill_invalid(TrackOut* out)
{
    out->vx = out->vy = 0.0f;
    out->valid = 0;
    out->measured = 0;
    out->cx = out->cy = out->raw_cx = out->raw_cy = 0.0f;
    out->w = out->h = 0.0f;
    out->class_id = -1;
    out->lost = 0;
}

static int in_skip(const Tracker* t, float x, float y)
{
    const float r2 = t->skip_r * t->skip_r;
    for (int i = 0; i < t->skip_n; i++) {
        unsigned long age = (t->now_ms > t->skip_t[i])
                          ? (t->now_ms - t->skip_t[i]) : 0;
        if (age > t->skip_max_age_ms)
            age = t->skip_max_age_ms;
        float dt = (float)age * 0.001f;

        float dx = x - (t->skip_x[i] + t->skip_vx[i] * dt);
        float dy = y - (t->skip_y[i] + t->skip_vy[i] * dt);
        if (dx * dx + dy * dy < r2)
            return 1;
    }
    return 0;
}

static int pick_new(const Tracker* t, const YoloBox* dets, int nd)
{
    int best = -1;
    float best_area = 0.0f;
    for (int i = 0; i < nd; i++) {
        if (t->want_class != TRK_CLASS_ANY && dets[i].class_id != t->want_class)
            continue;
        float w = dets[i].right - dets[i].left;
        float h = dets[i].bottom - dets[i].top;
        if (w <= 0.0f || h <= 0.0f)
            continue;

        if (in_skip(t, (dets[i].left + dets[i].right) * 0.5f,
                       (dets[i].top + dets[i].bottom) * 0.5f))
            continue;
        float a = w * h;
        if (a > best_area) {
            best_area = a;
            best = i;
        }
    }
    return best;
}

static int pick_match(const Tracker* t, const YoloBox* dets, int nd)
{
    int best = -1;
    float best_d2 = t->gate * t->gate;
    for (int i = 0; i < nd; i++) {
        if (t->class_id >= 0 && dets[i].class_id != t->class_id)
            continue;
        float cx = (dets[i].left + dets[i].right) * 0.5f;
        float cy = (dets[i].top + dets[i].bottom) * 0.5f;
        float dx = cx - t->kf.x[0];
        float dy = cy - t->kf.x[1];
        float d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
            best_d2 = d2;
            best = i;
        }
    }
    return best;
}

void trk_update(Tracker* t, const YoloBox* dets, int nd,
                float dt, float lead_sec, TrackOut* out)
{
    if (t->active)
        kf_predict(&t->kf, dt);

    int idx = -1;
    if (t->active)
        idx = pick_match(t, dets, nd);
    if (idx < 0 && !t->active)
        idx = pick_new(t, dets, nd);

    if (idx >= 0) {
        const YoloBox* d = &dets[idx];
        float cx = (d->left + d->right) * 0.5f;
        float cy = (d->top + d->bottom) * 0.5f;
        float w  = d->right - d->left;
        float h  = d->bottom - d->top;

        if (!t->active) {
            kf_start(&t->kf, cx, cy);
            t->active   = 1;
            t->uid++;
            t->class_id = d->class_id;
            t->w = w;
            t->h = h;
            t->hits = 1;
        } else {
            kf_update(&t->kf, cx, cy);

            t->w += (w - t->w) * t->size_lpf;
            t->h += (h - t->h) * t->size_lpf;
            t->hits++;
        }
        t->lost = 0;
        fill_out(t, out, 1, lead_sec);
        return;
    }

    if (!t->active) {
        fill_invalid(out);
        return;
    }

    t->lost++;
    if (t->lost <= t->coast_max) {
        fill_out(t, out, 0, lead_sec);
        return;
    }

    if (t->lost > t->coast_max * 4) {
        t->active = 0;
        t->class_id = -1;
        t->kf.inited = 0;
        t->hits = 0;
    }
    fill_invalid(out);
}

void trk_coast(Tracker* t, float dt, float lead_sec, TrackOut* out)
{
    if (!t->active || !t->kf.inited) {
        fill_invalid(out);
        return;
    }

    t->lost++;
    if (t->lost > t->coast_max) {
        if (t->lost > t->coast_max * 4) {
            t->active = 0;
            t->class_id = -1;
            t->kf.inited = 0;
            t->hits = 0;
        }
        fill_invalid(out);
        return;
    }

    kf_predict(&t->kf, dt);
    fill_out(t, out, 0, lead_sec);
}

void trk_release(Tracker* t)
{
    t->active   = 0;
    t->class_id = -1;
    t->kf.inited = 0;
    t->lost     = 0;
    t->hits     = 0;
    t->w = t->h = 0.0f;
}

void trk_set_time(Tracker* t, unsigned long now_ms)
{
    t->now_ms = now_ms;
}

void trk_mark_done(Tracker* t, float x, float y, float vx, float vy,
                   float radius_px)
{
    if (vx * vx + vy * vy < t->vel_deadband * t->vel_deadband)
        vx = vy = 0.0f;

    if (radius_px > 0.0f)
        t->skip_r = radius_px;
    t->skip_x[t->skip_head] = x;
    t->skip_y[t->skip_head] = y;
    t->skip_vx[t->skip_head] = vx;
    t->skip_vy[t->skip_head] = vy;
    t->skip_t[t->skip_head] = t->now_ms;
    t->skip_head = (t->skip_head + 1) % TRK_MAX_SKIP;
    if (t->skip_n < TRK_MAX_SKIP)
        t->skip_n++;
}

void trk_clear_done(Tracker* t)
{
    t->skip_n = 0;
    t->skip_head = 0;
}
